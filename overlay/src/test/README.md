# 自定义测试工具

ROS 2 包名为 `aimdk_test`，使用 `ament_python`。当前保留头部 `head_test`
和左腕专项诊断 `arm_test`；默认只读，运动必须显式添加 `--execute`。

通用单次七关节/夹爪控制已整合到 `aimdk_functions dual_arm_control`，
用法见 [功能包 README](../functions/README.md)。旧 `dual_arm_control`、
`shoulder_direction_test` 和 `gripper_close` 在本包的入口与实现已删除。
迁移前的完整说明和用户更新的验证事实保存在 [历史说明](../../../docs/archive/dual_arm_legacy_usage.md)，
真机记录见 [历史实机记录](../../../docs/archive/dual_arm_test.md)；历史命令不代表当前入口。

`arm_test` 保留的理由：它检查 STAND_DEFAULT → URS 的接管、左腕小幅往返
以及站立交接；这些专项实验契约未被“预先进入 URS、单次去程退出”的新功能覆盖。
`arm_test_core.py`（无 ROS）和 `arm_test_motion.py` 继续服务该诊断。
头部脚本及其核心、日志模块没有随本次功能迁移修改。

由用户同步后在机器人开发板 Humble 容器内，从工作区根目录构建：

```bash
bash scripts/build_overlay.sh aimdk_test aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_test head_test --help
ros2 run aimdk_test arm_test --help
ros2 run aimdk_functions dual_arm_control --help
```

固定 underlay 使用其他安装目录时，构建前设置 `AIMDK_UNDERLAY_INSTALL`，运行时加载同一安装目录。
默认日志位于检测到的工作区 `log/`，无法探测时显式指定 `--log-dir`。
本次未执行机器人容器构建和真机测试。

旧安装入口可能受同步排除规则保护而残留。同步源码后需要重新构建；如仍能看到
已移除的命令，先在容器核实实际安装布局和 overlay 来源，再清理对应旧产物，
不要将旧入口残留当作迁移后功能，也不要为此修改现有同步排除配置。

新增实验模块放入 `aimdk_test/`，在 `setup.py` 注册入口，并在 `package.xml`
声明依赖。包内导入使用 `aimdk_test` 前缀。
维护者邮箱和许可证目前为占位信息，正式发布前需替换。
