"""原生上肢数学后端的包内适配；坐标统一为 torso_link、m、rad。

这里只定义数学调用，不导入 ROS，不发布指令。成组提取的后端安装在本包的
vendor 子包；不搜索系统 x2ik 或本机参考目录。
"""

from importlib import import_module

import numpy as np

from .parameters import apply_execution_limits, model_path, side_limits


def _array(value, shape, label):
    """将数值输入复制为指定形状的有限数组；格式错误时抛出 ValueError。"""
    try:
        array = np.asarray(value)
        if array.shape != shape or array.dtype.kind not in 'iuf':
            raise ValueError(f'{label} 必须为形状 {shape} 的数值数组')
        array = array.astype(float, copy=True)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} 必须为形状 {shape} 的数值数组') from exc
    if not np.isfinite(array).all():
        raise ValueError(f'{label} 包含非有限值')
    return array


def _rotation(value):
    """校验将局部坐标映射至父坐标的 SO(3) 矩阵；不接受四元数输入。"""
    rotation = _array(value, (3, 3), '旋转矩阵')
    # 这是浮点矩阵格式校验，不作为实测姿态精度门槛。
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0)):
        raise ValueError('旋转矩阵必须正交且行列式为 +1')
    return rotation


def tool_transform(translation_m=None, rotation_matrix=None):
    """返回腕到工具的平移及旋转副本；省略二者表示腕原点 none。

    平移在所选侧 wrist_roll_link 中表达，旋转将工具坐标映射到腕坐标。
    自定义工具必须同时提供两项，避免以平移拟合隐含替代方向标定。
    """
    if translation_m is None and rotation_matrix is None:
        return np.zeros(3), np.eye(3)
    if translation_m is None or rotation_matrix is None:
        raise ValueError('自定义 TCP 必须同时提供 translation_m 和 rotation_matrix')
    return _array(translation_m, (3,), 'TCP 平移/m'), _rotation(rotation_matrix)


def _backend_classes():
    """加载包内模型、求解与 TCP 后端；安装不完整时抛出 RuntimeError。"""
    prefix = __package__ + '.vendor'
    try:
        model_module = import_module(prefix + '.x2_arm_model')
        solver_module = import_module(prefix + '.x2_srs_ik')
        tcp_module = import_module(prefix + '.x2_tcp')
    except ModuleNotFoundError as exc:
        if exc.name == prefix or exc.name.startswith(prefix + '.'):
            raise RuntimeError(
                '包内 FK/IK 后端安装不完整：需要 x2_arm_model、x2_srs_ik、'
                'x2_srs_batch、x2_tcp 及 TCP 资源；请核对 aimdk_functions 安装。'
            ) from exc
        raise
    return model_module.ArmModel, solver_module.SrsArmIK, tcp_module


class ArmKinematics:
    """为单侧模型提供显式初解与跟踪接口，不自动提交连续求解状态。"""

    def __init__(self, side, tcp_mode='none', tcp_file=None, tcp_config=None):
        """以官方模型几何、参考限位及肩roll调整构造单侧后端；side 仅接受 left/right。

        tcp_mode 默认 none（腕原点）；gripper/hand 为参考资源的名义工具点，
        不表示已实测标定。custom 需通过 tcp_file 或 tcp_config 提供双侧
        完整工具变换。缺少后端时抛出 RuntimeError，非法输入抛出 ValueError。
        不读取参考 MJCF 力矩范围。
        """
        self.side = side
        self.limits = np.asarray(side_limits(side), dtype=float)
        model_class, solver_class, tcp_module = _backend_classes()
        self.tool = tcp_module.load_tcp_tools(
            mode=tcp_mode, tcp_file=tcp_file, tcp_config=tcp_config,
        )[side]
        offset, rotation = tool_transform(
            self.tool['translation_m'], self.tool['rotation_matrix'],
        )
        self.model = model_class(
            side=side, urdf_path=model_path(), tcp_offset=offset,
            tcp_rotation=rotation, mjcf_path=None,
        )
        # ArmModel 的 FK 批量表只依赖几何；IK 可达范围及批量后端稍后才构造。
        apply_execution_limits(self.model)
        self.solver = solver_class(self.model)

    def _joints(self, value, require_limits):
        """校验七维 rad 向量；按需检查执行范围，始终不裁剪。"""
        joints = _array(value, (7,), '单臂关节/rad')
        if require_limits and (np.any(joints < self.limits[:, 0])
                               or np.any(joints > self.limits[:, 1])):
            raise ValueError(f'{self.side} 关节目标或求解种子超出新执行器有效范围')
        return joints

    def forward(self, joints):
        """从七维反馈计算 torso_link 下 TCP 位姿；超执行范围反馈也可观测。

        返回 (位置/m, 旋转矩阵)，矩阵将 TCP 坐标映射至 torso_link。
        本接口仅要求反馈有限，不将反馈值裁剪到执行范围。
        """
        return self.model.forward_kinematics(self._joints(joints, False))

    def _candidate(self, solution):
        """校验候选有限性与统一限位；无解保留 None，不替换或裁剪候选。"""
        if solution is not None:
            self._joints(solution.q, True)
        return solution

    def solve_initial(self, position_m, rotation_matrix, seed):
        """以明确种子求初解，返回上游候选或 None；不投影请求位姿。

        保留初始求解的数值精修与数值兜底；候选残差、SEW 和分支由后续
        验证层独立记录和评估，此处不宣称已达到实体定位精度。
        """
        position = _array(position_m, (3,), 'TCP 目标位置/m')
        rotation = _rotation(rotation_matrix)
        joints = self._joints(seed, True)
        return self._candidate(self.solver.solve(
            position, rotation, q_seed=joints, fallback_numeric=True,
        ))

    def track(self, position_m, rotation_matrix, previous_joints,
              previous_psi=None, previous_branch=None):
        """从上一已接受目标跟踪一帧，禁用工作空间投影及全局搜索兜底。

        返回上游候选或 None；调用者须独立检查残差、步长和分支并决定是否
        接受。失败不会推进任何内部关节、SEW 或分支状态。
        """
        position = _array(position_m, (3,), 'TCP 目标位置/m')
        rotation = _rotation(rotation_matrix)
        joints = self._joints(previous_joints, True)
        if previous_psi is not None and not np.isfinite(previous_psi):
            raise ValueError('上一已接受 SEW 臂角必须有限')
        return self._candidate(self.solver.track(
            position, rotation, q_prev=joints, psi_prev=previous_psi,
            branch_prev=previous_branch, clamp_reach=False, fallback=False,
        ))
