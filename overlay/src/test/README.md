# 自定义测试工具

ROS 2 包名为 `aimdk_test`，使用 `ament_python`。提供头部 `head_test`、
左腕专项诊断 `arm_test` 和持续发布分段实验 `upper_body_sequence_test`；
默认只读，运动必须显式添加 `--execute`。

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

## 持续发布的两段平移实验

`upper_body_sequence_test` 用于区分重新设定位置参考与停止发布对启动下沉的影响。
适用X2 Ultra、v1.3.0模型、机器人Humble容器；新增实现尚未完成容器构建及真机验证。
依赖 `aimdk_functions` 的运动规划、原生消息、反馈及日志组件，构建时选择两个包。

由用户预先进入URS/RUNNING后执行；脚本只查询模式。默认双臂各执行两段torso_link系
X+20mm，每段5秒；第一段终点确认后持续发送同一终点1秒，再进入第二段。
TCP为腕原点，保持起始姿态，采用端点IK和关节五次插值，途中TCP不保证直线。
自研位置偏置补偿固定关闭，夹爪开度保持最初采样目标。

```bash
# 在机器人Humble容器中构建、加载；只读预览
bash scripts/build_overlay.sh aimdk_functions aimdk_test
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_test upper_body_sequence_test --side both

# 对照组：第二段从第一段命令终点继续规划，理想累计X+40mm
ros2 run aimdk_test upper_body_sequence_test --side both \
  --delta-mm 20 --duration 5 --hold 1 --second-origin command --execute

# 重采样组：第二段从保持结束时所选臂的实际反馈重新规划X+20mm
ros2 run aimdk_test upper_body_sequence_test --side both \
  --delta-mm 20 --duration 5 --hold 1 --second-origin feedback --execute

# 零增量对比：分别使用command与feedback，观察第二段是否重新触发下沉
ros2 run aimdk_test upper_body_sequence_test --side both \
  --delta-mm 0 --hold 1 --second-origin feedback --execute
```

`--side`支持left/right/both，`--axis`支持x/y/z；`--duration`为每段5..10秒，
`--hold`为第一段终点确认后的非负保持秒数。`command`分支提前完成两段规划；
`feedback`分支在保持结束时仅重新采样所选臂，未选臂和夹爪继续使用原目标，
随后后台求解第二段，主循环继续发送第一段终点，实际保持时间包含求解等待。
`--timeout`默认5秒、范围1..10秒，也用于后台规划等待；超时停止发布，等待求解线程结束后退出。
只读模式下，feedback第二段以第一目标作暂定预览，实际执行时重新采样求解。

复用现有50Hz发布、80ms发布间隔上限、反馈错误锁存和时间推进检查。
每段终点沿用±5°关节容差，轨迹结束后额外最多2秒内连续确认0.3秒；
确认期间继续发布终点。完成第二段、中断或异常后停止发布，不自动返程或切换模式。
退出后的保持行为仍由原厂控制器决定。

验收时检查日志中的 `segment_begin`、`segment_confirmed`、`hold_begin/end`、
`second_origin_capture`、`second_plan` 和全部 `command_published`、原始反馈：
两段之间发布持续、保持时目标不变，第二段起点符合所选来源；
比较第二段首帧前后反馈，判断重采样分支是否出现新增下沉。
零增量command分支应全程保持同一完整目标，作为持续发布基线。
原始反馈、源时间和接收时间均保留；日志默认写入工作区 `log/`，也可用 `--log-dir`指定。
关节终点确认不能作为实际TCP移动20mm的精度结论，需结合反馈FK及实体测量评价。
