# 自定义实验与诊断工具

ROS 2包名为`aimdk_test`，使用`ament_python`。当前保留：

| 命令 | 用途 |
| --- | --- |
| `head_test` | 头部诊断与运动实验，默认只读，显式`--execute`才执行 |
| `upper_body_command_watch` | 只读观察原生上肢命令与MC的HAL手臂位置命令 |

上肢关节、TCP及夹爪控制使用`aimdk_functions upper_body_control`，见[功能包说明](../functions/README.md)。
旧左腕实验`arm_test`及两段平移实验`upper_body_sequence_test`已完成并移除。头部实验继续使用`head_test_core.py`及`head_test_diagnostics.py`。

## 同步与构建

由用户同步后在机器人Humble容器的工作区根目录执行，流程见[根目录说明](../../../README.md)：

```bash
bash scripts/build_overlay.sh aimdk_functions aimdk_test
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_test head_test --help
ros2 run aimdk_test upper_body_command_watch --help
ros2 run aimdk_functions upper_body_control --help
```

默认日志位于检测到的工作区`log/`，无法探测时显式指定`--log-dir`。
同步排除规则保护构建和安装产物，源码删除不会清理旧模块；容器内核实实际安装布局及包来源后，定向清理已移除的旧入口和模块，重新构建并检查`ros2 pkg executables aimdk_test`仅列出上表两个入口。
本次清理仅完成本地静态检查，尚未在机器人容器构建或验证。

## 只读检查上肢命令来源

`upper_body_command_watch` 用于观察 URS 接管前后的命令来源，在机器人 Humble 容器中运行。
每秒查询 `/mc/upper_body_command` 和 `/aima/hal/joint/arm/command` 的发布端点，
记录节点名称、命名空间、消息类型、GID 和 reliability/durability/history/depth QoS；
端点变化时输出到终端。首条消息完整显示，每5秒输出接收数量和窗口接收频率。
两话题的每条已接收消息均完整写入 JSONL，保留原始头部和字段、接收单调时间及 ROS 时间。

```bash
bash scripts/build_overlay.sh aimdk_test
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_test upper_body_command_watch --duration 60
```

先在已进入 URS 的状态下启动观察，再在另一个已加载相同环境的终端启动运动实验，
用实验日志与观察日志的时间戳对照接管前后变化。默认观察30秒，`--duration 0` 持续到 Ctrl-C。
日志默认位于工作区 `log/upper_body_*.jsonl`，metadata.command 为 `upper_body_command_watch`；
可用 `--log-dir` 指定目录。新增入口需要用户同步后在容器重新构建。

脚本创建两个只读订阅，使用 BEST_EFFORT/VOLATILE、depth=100；记录从订阅建立后实际收到的消息，
接收频率不作为发布频率或无丢包证明。发布端点按秒轮询，可能遗漏短暂出现的发布者。
没有消息表示本次观察未收到；话题端点和消息是否代表原厂控制器内部生效目标，需要结合真机结果核实。
发布者列表与消息分别记录，消息未逐条归属到具体发布端点；原生消息的 source 字段保留原值。
本地仅进行语法和静态检查，尚未在机器人容器运行验证。

## URS 首帧下沉验证结论

2026-11-02，依据[命令来源分析](../../../exports/upper_body_command_watch_20261102/report.md)，
将上半身控制的双臂规划起点由实际关节反馈改为MC发布的
`/aima/hal/joint/arm/command`中的`position`，按关节名称映射为原生`arm_pos`。
用户完成4次真机测试，首帧均未观察到下沉。

结论：以HAL命令的`position`初始化已解决本次验证中的首帧下沉问题，
作为`upper_body_control`的默认手臂起点；参数及行为见[功能包说明](../functions/README.md#手臂规划起点)。
