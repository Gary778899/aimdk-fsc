"""单臂直线离散IK及显式执行入口，夹爪状态门槛按本轮约定屏蔽。"""

import sys

from .upper_body_move import main as run_motion


def main(args=None):
    """复用原生反馈和执行生命周期，选择完整直线规划；返回进程退出码。"""
    return run_motion(args, line=True)


if __name__ == '__main__':
    sys.exit(main())
