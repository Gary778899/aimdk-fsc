"""独立肩 pitch 方向验证入口：单侧小幅往返，不主动开闭夹爪。"""

import sys

from aimdk_test.dual_arm_control import main as control_main


def main(args=None):
    """调用单侧肩 pitch 验证流程，返回控制入口的退出码。"""
    return control_main(args, probe=True)


if __name__ == '__main__':
    sys.exit(main())
