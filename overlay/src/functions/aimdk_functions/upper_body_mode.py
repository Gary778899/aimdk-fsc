"""原生上肢执行前的有界 URS/RUNNING 查询，不切换模式。"""

import time

from .dual_arm_control_core import URS, ControlFailure


def accept_mode(response, diagnostics):
    """校验启动时的 GetMcAction 响应；非 URS/RUNNING 时提示用户并失败。"""
    if response is None:
        raise ControlFailure('get_mc_action 未返回响应')
    action, status = response.info.action_desc, response.info.status.value
    diagnostics.record('mode_response', code=response.header.code, action=action, status=status)
    if response.header.code != 0:
        raise ControlFailure(f'get_mc_action 查询失败，code={response.header.code}')
    if action != URS or status != 100:
        raise ControlFailure(f'当前模式为 {action} / status={status}；'
                          '请先手动切换至 URS（UPPERBODY_REMOTE_SPLIT），等待 RUNNING 后重新运行')


def confirm_initial_mode(rclpy, node, options, diagnostics, client):
    """用给定客户端在启动时确认 URS；请求每 0.25 秒有界重试。

    服务发现和响应各有 options.timeout 秒预算。非 URS、错误响应或 ROS
    停止时直接抛出 ControlFailure，不将其视为可重试的响应超时。
    """
    from aimdk_msgs.srv import GetMcAction
    future = None
    began = time.monotonic()
    discovery_deadline = began + options.timeout
    try:
        while not client.service_is_ready():
            if not rclpy.ok():
                raise ControlFailure('ROS 已停止，无法查询 get_mc_action')
            if time.monotonic() >= discovery_deadline:
                raise ControlFailure('get_mc_action 服务发现超时')
            rclpy.spin_once(node, timeout_sec=.01)
        diagnostics.record('mode_service_ready', discovery_seconds=time.monotonic() - began)
        deadline = time.monotonic() + options.timeout
        attempt = 0
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise ControlFailure('ROS 已停止，无法查询 get_mc_action')
            attempt += 1
            sent = time.monotonic()
            request = GetMcAction.Request()
            request.request.header.stamp = node.get_clock().now().to_msg()
            future = client.call_async(request)
            diagnostics.record('mode_query_sent', phase='startup', attempt=attempt)
            attempt_deadline = min(deadline, sent + .25)
            while not future.done() and time.monotonic() < attempt_deadline:
                if not rclpy.ok():
                    raise ControlFailure('ROS 已停止，无法等待 get_mc_action 响应')
                rclpy.spin_once(node, timeout_sec=.01)
            if not rclpy.ok():
                raise ControlFailure('ROS 已停止，无法确认 get_mc_action 响应')
            if future.done() and time.monotonic() < attempt_deadline:
                diagnostics.record('mode_query_received', phase='startup', attempt=attempt,
                                    response_seconds=time.monotonic() - sent)
                return accept_mode(future.result(), diagnostics)
            # 仅重试只读查询；移除旧请求，迟到响应不能确认后续的模式归属。
            client.remove_pending_request(future)
            future = None
            diagnostics.record('mode_query_retry', phase='startup', attempt=attempt,
                                response_wait_seconds=time.monotonic() - sent)
        raise ControlFailure(f'get_mc_action 响应超时：{options.timeout:g} 秒内已发送 {attempt} 次查询')
    finally:
        if future is not None and not future.done():
            client.remove_pending_request(future)
