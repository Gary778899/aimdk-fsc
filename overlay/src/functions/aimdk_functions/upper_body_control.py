"""X2 Ultra统一单次上肢控制：joint/tcp/gripper，默认只读，显式--execute发布。

限位约定：统一采用参考项目x2ik 2.1.0的关节范围，左肩roll下限减1°、右肩roll上限加1°；
唯一数值定义为upper_body_kinematics.parameters.EXECUTION_LIMITS_RAD，不与旧SDK范围取交集。
TCP暂按法兰面中心等同wrist_roll_link腕原点处理，实体验证列入待办。
"""

import argparse
import math
import os
import sys
import time

from .dual_arm_control_core import ARM_NAMES, JOINT_TAGS, ControlFailure, check_arm, interpolate
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_control_core import ArmCompensationGroup, plan_motion, selected_sides
from .upper_body_diagnostics import UpperBodyLog
from .upper_body_execution import UpperBodyExecutor
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_kinematics.parameters import parameter_report
from .upper_body_kinematics_check import code_manifest, log_directory
from .upper_body_mode import confirm_initial_mode


def parser():
    """创建三个独立子命令；手臂默认补偿、TCP默认直线，夹爪不提供补偿参数。"""
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest='action', required=True)
    for action in ('joint', 'tcp', 'gripper'):
        command = commands.add_parser(action, help={'joint': '单关节增量', 'tcp': 'TCP平移',
                                                   'gripper': '夹爪开度'}[action])
        command.add_argument('--side', choices=('left', 'right', 'both'), required=True)
        command.add_argument('--execute', action='store_true', help='操作者预先进入URS/RUNNING后显式执行')
        command.add_argument('--timeout', type=float, default=5., help='启动/服务/匹配等待1..10秒；夹爪持续发送时间')
        command.add_argument('--feedback-timeout', type=float, default=.5, help='各路反馈时限0.2..1秒')
        command.add_argument('--log-dir', help='默认工作区log目录')
        command.add_argument('--load-description', default='unknown', help='实际姿态、工具和负载说明')
        if action == 'gripper':
            command.add_argument('--opening', type=float, help='共用开度0..1，0闭合、1全开')
            for side in ('left', 'right'):
                command.add_argument(f'--{side}-opening', type=float, help='双侧独立开度0..1')
            command.set_defaults(compensation=False)
            continue
        command.add_argument('--duration', type=float, default=5., help='单程5..10秒，默认5')
        command.add_argument('--compensation', action=argparse.BooleanOptionalAction, default=True,
                             help='位置偏置补偿默认开启，--no-compensation关闭')
        command.add_argument('--payload-mass-kg', type=float, default=0., help='共用附加载荷质量kg，默认0')
        command.add_argument('--payload-com-m', type=float, nargs=3, default=(0., 0., 0.),
                             help='共用负载质心，所选侧腕坐标系XYZ/m')
        for side in ('left', 'right'):
            command.add_argument(f'--{side}-payload-mass-kg', type=float, help='覆盖该侧附加载荷质量kg')
            command.add_argument(f'--{side}-payload-com-m', type=float, nargs=3, help='覆盖该侧腕系质心XYZ/m')
        if action == 'joint':
            command.add_argument('--tag', choices=JOINT_TAGS, required=True)
            command.add_argument('--delta-deg', type=float, help='共用带符号增量，绝对值不超过10度')
            for side in ('left', 'right'):
                command.add_argument(f'--{side}-delta-deg', type=float, help='双臂独立带符号增量/度')
        else:
            command.add_argument('--path', choices=('line', 'endpoint'), default='line')
            command.add_argument('--axis', choices=('x', 'y', 'z'), help='单轴位移方向，默认x')
            command.add_argument('--delta-mm', type=float, help='沿axis的共用带符号位移/mm')
            command.add_argument('--translation-mm', type=float, nargs=3, help='torso系共用XYZ位移/mm，保持起始姿态')
            for side in ('left', 'right'):
                command.add_argument(f'--{side}-translation-mm', type=float, nargs=3,
                                     help='双臂独立torso系XYZ位移/mm')
    return result


def resolve_requests(options, common, left, right):
    """将共用值或双侧独立值展开成侧别字典；缺失或混用输入抛出ControlFailure。"""
    sides = selected_sides(options.side)
    if common is not None:
        if left is not None or right is not None:
            raise ControlFailure('共用目标与左右独立目标不能混用')
        return {side: common for side in sides}
    if options.side != 'both' or left is None or right is None:
        raise ControlFailure('单侧须提供共用目标；双侧须提供共用目标或完整左右独立目标')
    return dict(left=left, right=right)


def validate_options(options):
    """校验输入并展开分侧目标/负载；沿用既有时长、10度增量和反馈时限。"""
    options.duration = options.timeout if options.action == 'gripper' else options.duration
    for name, lower, upper in (('timeout', 1., 10.), ('feedback_timeout', .2, 1.),
                               ('duration', 1. if options.action == 'gripper' else 5., 10.)):
        value = getattr(options, name)
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ControlFailure(f'--{name.replace("_", "-")}须为{lower:g}..{upper:g}秒')
    if options.action == 'tcp':
        common = options.translation_mm
        if options.delta_mm is not None:
            if common is not None:
                raise ControlFailure('--delta-mm与--translation-mm不能混用')
            common = [0., 0., 0.]
            common[('x', 'y', 'z').index(options.axis or 'x')] = options.delta_mm
        elif options.axis is not None:
            raise ControlFailure('--axis需要同时提供--delta-mm')
        options.requests = resolve_requests(options, common, options.left_translation_mm,
                                             options.right_translation_mm)
        if not all(math.isfinite(v) for values in options.requests.values() for v in values):
            raise ControlFailure('TCP位移须为三个有限毫米数值')
    else:
        name = 'delta_deg' if options.action == 'joint' else 'opening'
        options.requests = resolve_requests(options, getattr(options, name),
                                             getattr(options, 'left_' + name),
                                             getattr(options, 'right_' + name))
        for value in options.requests.values():
            if not math.isfinite(value) or (abs(value) > 10 if options.action == 'joint' else not 0 <= value <= 1):
                raise ControlFailure('关节增量须为有限±10度；夹爪开度须为有限0..1')
    options.payloads = {}
    if options.action == 'gripper':
        return
    if (not math.isfinite(options.payload_mass_kg) or options.payload_mass_kg < 0
            or not all(math.isfinite(v) for v in options.payload_com_m)):
        raise ControlFailure('共用负载质量须为有限非负kg，质心须为三个有限m数值')
    for side in selected_sides(options.side):
        mass = getattr(options, side + '_payload_mass_kg')
        com = getattr(options, side + '_payload_com_m')
        mass = options.payload_mass_kg if mass is None else mass
        com = options.payload_com_m if com is None else com
        if not math.isfinite(mass) or mass < 0 or not all(math.isfinite(v) for v in com):
            raise ControlFailure('负载质量须为有限非负kg，质心须为三个有限m数值')
        options.payloads[side] = dict(mass=mass, com=com)
    for side in ('left', 'right'):
        if side not in options.payloads and (getattr(options, side + '_payload_mass_kg') is not None
                                            or getattr(options, side + '_payload_com_m') is not None):
            raise ControlFailure('不能指定未选侧负载')
    payload_given = (options.payload_mass_kg != 0 or any(options.payload_com_m)
                     or any(getattr(options, side + '_payload_' + suffix) is not None
                            for side in ('left', 'right') for suffix in ('mass_kg', 'com_m')))
    if not options.compensation and payload_given:
        raise ControlFailure('负载参数需要开启compensation')


def preview_motion(trajectory, options, compensation, values, diagnostics):
    """按统一20ms网格预览完整轨迹及补偿；任何候选越限在创建发布器前拒绝。"""
    from .upper_body_control_core import validate_target

    count = int(math.ceil(options.duration / .02))
    for index in range(count + 1):
        seconds = options.duration * index / count
        ideal = trajectory.at(seconds)
        target, details = ideal, None
        if compensation is not None:
            target, details = compensation.apply(trajectory.captured, ideal, values,
                                                  interpolate((0.,), (1.,), seconds, options.duration)[0])
        validate_target(trajectory.captured, target, options)
        diagnostics.record('plan_point', index=index, time_s=seconds, ideal_arm_rad=list(ideal.arm_rad),
                           sent_arm_rad=list(target.arm_rad), hand_openings=list(target.hand_openings),
                           compensation=details)
    print(f'预览终点/rad={list(target.arm_rad)}；夹爪={list(target.hand_openings)}', flush=True)


def main(args=None):
    """只读规划或显式执行一次动作；返回0完成、1失败、130中断，不自动返程或切换模式。"""
    options = parser().parse_args(args)
    diagnostics = node = port = client = None
    initialized = False
    exit_code, reason = 1, 'failed'
    try:
        validate_options(options)
        diagnostics = UpperBodyLog(log_directory(options.log_dir), command='upper_body_control',
                                   options=vars(options), parameters=parameter_report(),
                                   code_sha256=code_manifest(), python=sys.version,
                                   ros_distro=os.environ.get('ROS_DISTRO', 'unknown'),
                                   sdk_version='unknown', firmware_version='unknown',
                                   tcp='wrist_origin_assumed_flange_center',
                                   compensation_enabled=options.compensation,
                                   hand_state_check=False, hand_fault_check=True)
        print(f'日志：{diagnostics.path}', flush=True)
        print(f'动作={options.action}，侧别={options.side}，补偿={options.compensation}，'
              f'时长={options.duration:g}s；夹爪状态/静止门槛关闭，故障检查保留。', flush=True)
        arms = {side: ArmKinematics(side, tcp_mode='none') for side in selected_sides(options.side)
                if options.action == 'tcp' or options.compensation}
        compensation = None
        if options.compensation:
            from .upper_body_compensation import PositionCompensation

            calculators = {side: PositionCompensation(arm, options.payloads[side]['mass'],
                                                      options.payloads[side]['com'])
                           for side, arm in arms.items()}
            compensation = ArmCompensationGroup(calculators)
            diagnostics.record('compensation_config', arms={side: calc.describe()
                                                            for side, calc in calculators.items()})
            print('补偿刚度40 N·m/rad，偏置限幅±12°，随运动五次进度从零引入。', flush=True)
        import rclpy
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('upper_body_control')
        if node.get_parameter('use_sim_time').value:
            raise ControlFailure('本命令仅用于真机，不支持use_sim_time')
        if options.execute:
            from aimdk_msgs.srv import GetMcAction

            client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
            confirm_initial_mode(rclpy, node, options, diagnostics, client)
        port = UpperBodyExecutor(rclpy, node, options, diagnostics)
        port.compensation = compensation
        values = port.capture()
        captured = CapturedTarget(values['arm'].positions, values['hand'].positions)
        trajectory, reports = plan_motion(captured, options, arms)
        diagnostics.record('plan', kind=trajectory.kind, arms=reports,
                           start_arm_rad=list(captured.arm_rad), target_arm_rad=list(trajectory.target.arm_rad),
                           hand_openings=list(trajectory.target.hand_openings))
        for side, line in trajectory.lines.items():
            for point in line.points:
                diagnostics.record('line_plan_point', side=side, **point)
        preview_motion(trajectory, options, compensation, values, diagnostics)
        print(f'规划类型={trajectory.kind}；TCP使用torso_link坐标，腕原点暂视为法兰中心。', flush=True)
        if options.execute:
            refreshed = port.capture(refreshed_after=time.monotonic())
            check_arm(refreshed['arm'].positions, captured.arm_rad)
            final = port.execute(captured, trajectory.target, trajectory)
            fk = {}
            for side, arm in arms.items():
                offset = 0 if side == 'left' else 7
                position, rotation = arm.forward(final['arm'].positions[offset:offset + 7])
                fk[side] = dict(position_m=position.tolist(), rotation_matrix=rotation.tolist())
            diagnostics.record('execution_result', actual_arm_rad=list(final['arm'].positions),
                               ideal_target_rad=list(trajectory.target.arm_rad),
                               sent_arm_rad=list(port.last_sent.arm_rad), feedback_fk=fk,
                               hand_target=list(trajectory.target.hand_openings),
                               actual_hand_openings=list(final['hand'].positions))
            for name, origin, actual, expected in zip(ARM_NAMES, captured.arm_rad,
                                                     final['arm'].positions, port.last_sent.arm_rad):
                print(f'{name}: 实际增量={math.degrees(actual-origin):+.3f}°，'
                      f'发送目标误差={math.degrees(actual-expected):+.3f}°', flush=True)
            print(f'反馈FK={fk}；夹爪目标={trajectory.target.hand_openings}，'
                  f'实测={final["hand"].positions}；不判定夹爪到位。', flush=True)
            print('完成既有±3°关节确认，已停止发布；不自动返程，不切换模式。停止发布不等于急停。', flush=True)
        else:
            print('只读计划完成；显式--execute才执行一次单程。', flush=True)
        exit_code, reason = 0, 'completed'
    except KeyboardInterrupt:
        exit_code, reason = 130, 'interrupted'
        if port is not None:
            port.stop()
        print('用户中断；停止发布，不返程、不切换模式。', file=sys.stderr)
    except Exception as error:
        if port is not None:
            port.stop()
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        print(f'失败：{error}', file=sys.stderr)
    finally:
        try:
            if port is not None:
                port.close()
            if client is not None:
                node.destroy_client(client)
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
        finally:
            if diagnostics is not None:
                diagnostics.close(reason=reason)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
