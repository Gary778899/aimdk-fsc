# 自定义功能工具

ROS 2 包名为 `aimdk_functions`，使用 `ament_python`。目前提供音量命令
`volume_control` 和单次上肢控制命令 `dual_arm_control`。

由用户同步工作区后，在机器人开发板的 Humble 容器中，从工作区根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to aimdk_functions aimdk_test
source install/local_setup.bash
ros2 run aimdk_functions dual_arm_control --help
ros2 run aimdk_functions volume_control get-volume
```

如使用自定义构建、安装目录，沿用对应 colcon 参数并 source 对应安装目录。
本次功能整合只完成本地静态检查，尚未执行机器人容器构建或真机验收。
原有实验已验证的结果见 [test.md](../../test.md)，不自动推广为新增七关节动作的验收结果。

## 单次双臂与夹爪控制

入口是 `aimdk_functions/dual_arm_control.py`。核心校验与五次插值位于
`dual_arm_control_core.py`（无 ROS 导入），JSONL 日志位于
`dual_arm_control_diagnostics.py`。功能包不导入或依赖 `aimdk_test`。

先由操作者进入 `UPPERBODY_REMOTE_SPLIT`（URS），等待 `RUNNING`。
命令在启动时有界查询 GetMcAction，只接受 URS/RUNNING；之后使用 MC 状态话题
持续校验站立及模式，不调用 SetMcAction，不控制腰腿，不停止原生 MC。
每次命令采集 3 秒有效反馈并记录完整双臂起点及两侧夹爪开度。
默认只读，不创建命令发布器；只有显式 `--execute` 才执行一次动作。

执行采用 `/mc/upper_body_command`，所有命令始终包含左臂 7 + 右臂 7 的完整
14 关节 rad 目标及模式 1 的两侧夹爪开度。未选关节固定重发本次采样位置，
未选夹爪重发采样开度；不填零、不每帧用测量值覆盖保持目标。
夹爪开度是位置目标，不是夹持力保持。本轮沿用空爪动作检查，堵转、物体掉落或故障会中止。
关节动作启动前仍要求两爪静止。执行期间重发保持开度可能使夹爪短暂调整，
因此允许正常状态 0/1 下的速度或移动态变化；
故障、异常状态及反馈时效检查保持有效。终点仍要求夹爪为
`state=0` 且 `abs(velocity)<=0.01`，与关节到位一起连续确认 0.3 秒；
未重新稳定会按既有终点超时失败，不增加额外等待阶段或放宽速度阈值。
此调整依据 2026-10-27 三次关节命令因保持开度重发后的夹爪速度瞬态立即中止的记录，
仅完成本地静态检查，待机器人容器验证。
夹爪归一化开度与实际开合角度的对应关系尚未确认，已取消执行期间和终点的
开度误差门槛。目标及实测开度仅记录，不据此中止或宣称达到目标开度。
执行前由操作者确认动作空间及没有其他节点同时发布上肢目标。

每次命令只执行一次去程或一次夹爪开度控制，满足关节到位及夹爪静止确认后停止发布并退出。
正常结束、失败或 Ctrl+C 均不自动回程、不切换模式；停止发布不是急停，
退出后的持续保持与固件超时行为不由该程序保证。

### 关节 tag 与增量

| SDK 关节 | tag | 左/右 `arm_pos` 索引 |
| --- | --- | --- |
| J1 肩 pitch | `shoulder_pitch` | 0 / 7 |
| J2 肩 roll | `shoulder_roll` | 1 / 8 |
| J3 肩 yaw | `shoulder_yaw` | 2 / 9 |
| J4 肘 | `elbow` | 3 / 10 |
| J5 腕 yaw | `wrist_yaw` | 4 / 11 |
| J6 腕 pitch | `wrist_pitch` | 5 / 12 |
| J7 腕 roll | `wrist_roll` | 6 / 13 |

`joint` 必须指定 `--side`、`--tag` 和带符号的角度增量。
增量相对本次执行前采样起点，每个被选关节的 `|delta_deg| <= 10°`；
拒绝非有限值和超限输入，不截断、不拆成多段指令。正负号对应关节坐标，
不将同号自动解释为左右镜像动作。允许 0 增量，但不宣称发生实际运动。

单侧使用 `--delta-deg`；双侧可使用共用 `--delta-deg`，或同时指定
`--left-delta-deg` 与 `--right-delta-deg`。共用与独立参数不能混用，
独立参数仅适用于 `--side both`，且必须同时给出两侧。
两侧共用同一轨迹时间，采用起终点速度、加速度为零的五次插值。
`--duration` 默认 5 秒，可设置 5..10 秒；这是既有实验时间范围，不是厂商额定速度。

```bash
# 先只读检查左腕 roll 的 +2°计划。
ros2 run aimdk_functions dual_arm_control joint --side left --tag wrist_roll --delta-deg 2

# 左腕 roll 从本次实测位置增加 2°，单程 5 秒，不自动返回。
ros2 run aimdk_functions dual_arm_control joint --side left --tag wrist_roll --delta-deg 2 --execute

# 双肩 pitch 同时减少 5°。
ros2 run aimdk_functions dual_arm_control joint --side both --tag shoulder_pitch --delta-deg -5 --execute

# 双臂同名关节分别使用不同增量，同时起止。
ros2 run aimdk_functions dual_arm_control joint --side both --tag wrist_yaw \
  --left-delta-deg 2 --right-delta-deg -2 --duration 5 --execute
```

### 关节限位依据

范围参考随 SDK 提供的 [1.7.1 手臂可保证活动空间](../../docs/cn/_build/html/zh-cn/dev/about_agibot_X2/joint_name_and_limit.html)
以及官方 [motocontrol.py](../py_examples/py_examples/motocontrol.py) 的各侧关节定义。
文档角度值按 0.5°舍入，且未分别列左右臂；右肩 roll、右腕 roll 的符号映射
依据官方示例。程序取两份范围的交集，不附加旧肩实验中的 1°余量，也不将 HAL
示例的命令通道、增益或 Ruckig 依赖带入本功能。

| tag | 左臂有效范围 / ° | 右臂有效范围 / ° |
| --- | --- | --- |
| `shoulder_pitch` | -176.471 ~ 116.5 | 相同 |
| `shoulder_roll` | -3.495 ~ 171.486 | -171.486 ~ 3.495 |
| `shoulder_yaw` | -146.448 ~ 146.448 | 相同 |
| `elbow` | -134.966 ~ 0 | 相同 |
| `wrist_yaw` | -146.448 ~ 146.448 | 相同 |
| `wrist_pitch` | -30 ~ 30 | 相同 |
| `wrist_roll` | -86.5 ~ 41.482 | -41.482 ~ 86.5 |

上表为便于阅读的近似值；程序直接使用文档度数转 rad 与示例 rad 的交集。
仅对本次选中关节检查起点、目标两端；五次插值在两端之间单调，不绕角。
未选关节原样保持，不因其与参考限位不符而自动裁剪或阻止无关动作。
历史实测肩 roll 曾超出参考边界：若该关节被选中，程序将明确拒绝动作并打印起点、
目标及范围，需核实机型/固件及限位定义，不自动放宽或执行越界回归。
上述关节范围不构成碰撞检查，也不表示已覆盖所有姿态与负载的验收。

### 夹爪开度

`gripper` 必须指定 `--side` 和 `--opening`（有限值 0..1）；0 闭合、1 全开，
中间值为部分张开。它不接受关节 tag、角度增量或 `--duration`。
更新夹爪开度目标后持续发布，等待夹爪静止确认；不生成手臂轨迹。
静止确认只说明发布后收到的新反馈满足静止条件，不证明夹爪达到目标开度或已发生预期运动。

```bash
# 默认只读，查看右爪半开计划。
ros2 run aimdk_functions dual_arm_control gripper --side right --opening 0.5

# 右爪半开，左爪保持采样开度，双臂保持本次起点。
ros2 run aimdk_functions dual_arm_control gripper --side right --opening 0.5 --execute

# 两爪全开；将 1 改为 0 即为两爪闭合。
ros2 run aimdk_functions dual_arm_control gripper --side both --opening 1 --execute
```

### 反馈、到位与日志

沿用已验证执行器的默认阈值：目标发布 50 Hz（20 ms 定时器）、发布间隔上限
80 ms、全部关节跟踪及终点误差 ±3°、终点连续确认 0.3 秒；不设置夹爪开度误差容差。
要求本阶段发布后收到更新且源时间推进的反馈；关节单程结束后额外等候最多 2 秒，
夹爪静止确认等候使用 `--timeout`，默认 5 秒、范围 1..10 秒。
该参数也用于启动服务发现/响应、反馈启动等待及 DDS 匹配的各自有界等待。
DDS 匹配和发布计数仅证明本地连接与发布，不能证明固件接受或执行命令。

MC、手臂和夹爪的接收间隔、源时间停滞及源年龄上限均默认 0.5 秒，
分别由 `--mc-feedback-timeout`、`--arm-feedback-timeout`、
`--hand-feedback-timeout` 设置（0.2..1 秒）。观察与执行使用相同监视器和阈值，
保留源时间回退、未来时间戳、ROS 时钟停滞/回退、硬件故障及错误锁存处理。
只为命令填写当前 ROS 时间，不改写反馈源时间，不解释有歧义的 FSM 编码。
发布 QoS 沿用已验证的官方 Python 上肢示例默认深度 10，反馈订阅沿用
`BEST_EFFORT + TRANSIENT_LOCAL`，不在迁移时变更通信策略。

小角度动作沿用 ±3°到位容差，因此小于该容差的指令可能在实际运动很小时也满足判定。
程序打印每个关节的目标、实测、实际增量与误差，不把“容差内确认”解释为精确运动了
输入角度；本轮没有增加未经实测的更严格精度门槛。

日志默认写入检测到的工作区 `log/dual_arm_control_*.jsonl`，探测依据为
`src/functions/package.xml`，不依赖测试包。无法检测时用 `--log-dir`。
日志记录完整起点、目标、原始反馈、发布、模式查询和到位结果；最多缓存 20000 个事件，
退出时写盘并记录被丢弃的旧事件数，因此不是无限时长的完整录制器。
关节执行期间夹爪静止判定变化时额外记录 `hand_hold_status`，包含该次反馈的源时间、
接收时间、目标、开度、速度、状态与故障字段，不受普通反馈约 10 Hz 的日志节流影响。

## 旧入口迁移

以下实验入口及对应实现已移除：`aimdk_test dual_arm_control`、
`aimdk_test shoulder_direction_test`、`aimdk_test gripper_close`。
改用本文的 `aimdk_functions dual_arm_control joint/gripper`；旧固定肩抬升、
开闭夹爪和返程串联流程不再提供。
`head_test` 及 `arm_test` 的专项诊断留在测试包；后者的模式接管与交接试验不属于新命令。
历史说明与已验证事实保存在 [迁移前说明](../../docs/archive/dual_arm_legacy_usage.md)。

新增入口需要在机器人容器内重新构建。rsync 对安装产物的保护性排除不会清除旧安装入口，
仅同步源码或 symlink 安装不能保证旧命令消失。若 `ros2 pkg executables aimdk_test`
仍列出已删除入口，应在容器确认实际安装布局及 overlay 来源后清理这些旧产物再构建，
不要继续运行旧命令，也不要猜测安装路径或改动同步排除规则。

新增脚本在 `setup.py` 注册入口，并在 `package.xml` 声明依赖。
维护者邮箱和许可证目前为占位信息，正式发布前需替换。
