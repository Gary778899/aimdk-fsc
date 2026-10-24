"""独立夹爪开度入口：0 闭合，1 完全张开；保持双臂实测位置，默认只读。"""

import sys

from aimdk_test.dual_arm_control import main as control_main


def main(args=None):
    """调用独立夹爪开度流程，返回控制入口的退出码；保留 gripper_close 命令名。"""
    return control_main(args, close_gripper=True)


if __name__ == '__main__':
    sys.exit(main())
