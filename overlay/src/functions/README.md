# 自定义功能工具

ROS 2 包名为 `aimdk_functions`，使用 `ament_python`。目前提供音量命令
`volume_control`、单次上肢控制命令 `dual_arm_control`，以及只读验证命令
`arm_compensation_probe`、`upper_body_kinematics_check`，以及默认只读的
单臂端点运动命令`upper_body_move`。

由用户同步工作区后，在机器人开发板的 Humble 容器中，从工作区根目录构建：

```bash
bash scripts/build_overlay.sh aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_functions dual_arm_control --help
ros2 run aimdk_functions volume_control get-volume
```

固定 underlay 使用其他安装目录时，构建前设置 `AIMDK_UNDERLAY_INSTALL`，运行时加载同一安装目录。
本次功能整合只完成本地静态检查，尚未执行机器人容器构建或真机验收。
原有实验已验证的结果见 [历史实机记录](../../../docs/archive/dual_arm_test.md)，不自动推广为新增七关节动作的验收结果。

## SD / URS 重力补偿只读观测

脚本为 `aimdk_functions/arm_compensation_probe.py`。在机器人 Humble/AimDK
环境中使用；不需要 Pinocchio、MuJoCo 或额外 Python 第三方依赖。
由操作者预先进入待观测模式；tag 只指定预期模式，不调用 SetMcAction，
不发布运动指令，也不会停止原厂 MC。`SD` 对应 `STAND_DEFAULT`，
`URS` 对应 `UPPERBODY_REMOTE_SPLIT`（不是 `UPPER_REMOTE_SPLIT`）。

同步后可直接从工作区根目录运行源码，无需为此重新构建功能包；
终端应已加载机器人现有的 ROS 2 Humble 和匹配的 AimDK 环境：

```bash
python3 src/functions/aimdk_functions/arm_compensation_probe.py --tag SD

# 操作者按已有流程进入 URS 后，另行执行，不会自动切换模式。
python3 src/functions/aimdk_functions/arm_compensation_probe.py --tag URS

# 可延长记录并指定实际工作区内的日志目录。
python3 src/functions/aimdk_functions/arm_compensation_probe.py \
  --tag URS --seconds 30 --sample-hz 50 --log-dir ./log
```

按本页构建并加载安装环境后，也可以使用：

```bash
ros2 run aimdk_functions arm_compensation_probe --tag SD
ros2 run aimdk_functions arm_compensation_probe --tag URS
```

默认等待 ROS 图发现 3 秒，再正式记录 15 秒。每路话题最多保存 50 Hz，
实际订阅为 best_effort / volatile / depth=100；计数覆盖回调收到的所有样本，
不能统计 DDS 或订阅队列已经丢失的样本。源频率较低时不会补帧，
高频源会被限频保存，不代表全量实时追踪或严格同步。

每次在自动探测的工作区 `log/` 下创建独立 `arm_probe_<TAG>_<UTC时间>_<唯一值>/`：

| 文件 | 内容 |
| --- | --- |
| `metadata.json` | 预期模式、采集参数、主机、Python/ROS 环境及话题清单 |
| `graph_start.json` / `graph_end.json` | 采集前后节点、话题、服务和目标话题各端点的类型、GID、QoS |
| `events.jsonl` | 完整消息字段、接收墙钟/单调时间、模式观测及服务响应/超时 |
| `summary.json` | 各路收到/保存数量、源时间重复/回退/零值计数、模式匹配、缺失必需流和错误 |
| `report.log` | 可直接查看和回传的中文模式与话题统计报告 |

记录范围包括上半身目标、手臂/头/手/腰命令与反馈、胸部/躯干 IMU、
MC common state、通用 debug 和 RL debug。源消息中的时间、关节名称、
effort、增益、模式和错误字段完整保留；非有限浮点保存为 `NaN`、`inf`、
`-inf` 字符串。debug 数组不自动解释未知槽位。
GetMcAction 每秒尝试查询一次，每次最多等待 2 秒；同时记录 common state
的模式变化。不会自动生成“已启用/未启用重力补偿”的结论，也不从不同时间的
command/state 直接计算补偿力矩。`meas_stamp=0` 保留原值，不以接收时间替代。

实际模式不符时仍继续记录，并标记不匹配；无上半身目标发布者或某些可选话题
没有消息时也继续记录。退出码 `0` 表示已收到手臂 command/state、至少一次
GetMcAction 确认目标模式及 RUNNING、未观察到模式不符且未发生采集错误；
它不代表重力补偿或真机控制验收通过。退出码 `2` 表示观测不完整/模式不符，
`130` 表示 Ctrl+C，`1` 表示环境或文件错误；已产生的日志保留。
Ctrl+C 或异常时尽量写入总结；强制杀进程或断电可能只保留已写出的事件。
环境加载等启动错误尽量写入 `failure.json`。优先查看 `report.log`，
进一步分析使用 `events.jsonl` 的消息字段；只回传统计报告不能完成力矩分析。

日志可通过现有 VS Code 任务“机器人：下载日志”取回，先等待脚本退出。
无需修改同步排除规则。此工具仅完成本地静态/语法审阅，待机器人容器实测。

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
夹爪开度是位置目标，不是夹持力保持。
本轮依据 2026-10-29 三次夹爪测试在发布前因右爪持续 `state=2`、故障码为0而中止的记录，
按操作者要求屏蔽本命令（`joint` 和 `gripper`）的夹爪状态及静止门槛：
起点采样、执行循环和终点确认均不再以 `state` 或速度阻断。
状态、速度、开度仍记录；夹爪故障码、有限值、类型/数量和源时间/时效检查继续有效。
因此 `state=2/3` 本身不会中止，非零 `faultcode` 仍会中止。
终端及日志明确记录 `hand_state_check=False`；不宣称夹爪静止或达到目标开度。
夹爪归一化开度与实际开合角度的对应关系尚未确认，开度误差仍仅记录。
屏蔽目的是排除该门槛对命令发送的阻断，验证 URS 下原厂运控能否执行夹爪开合，
不将 `state=2` 重新解释为正常状态。
用户回传的四次实机记录（`20261029T125303`、`125321`、`125335`、`125349`）
分别完成左开、右开、右闭、左闭；每次发布265帧并通过既有双臂±3°确认。
所选夹爪归一化位置反馈分别由约0变为1、约1变为0，夹爪故障码均为0。
这些记录支持本轮开合命令及反馈变化的验证，不代表物理开合角度、夹持力或通用到位精度验收。
执行前由操作者确认动作空间及没有其他节点同时发布上肢目标。

每次命令只执行一次去程或一次夹爪开度控制，完成目标持续发送及关节到位确认后停止发布并退出。
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

范围参考随 SDK 提供的 [1.7.1 手臂可保证活动空间](../../../underlay/docs/cn/_build/html/zh-cn/dev/about_agibot_X2/joint_name_and_limit.html)
以及官方 [motocontrol.py](../../../underlay/src/py_examples/py_examples/motocontrol.py) 的各侧关节定义。
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
更新夹爪开度目标后按 `--timeout` 持续发布（默认 5 秒），不生成手臂轨迹。
随后按双臂保持目标及新源反馈连续确认 0.3 秒；不检查夹爪状态、静止或开度误差。
正常退出只表示命令发送及双臂保持检查完成，不证明夹爪发生预期运动或达到目标。

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
夹爪目标持续发送时间使用 `--timeout`，默认 5 秒、范围 1..10 秒；
发送阶段结束后额外等候关节确认最多 2 秒。
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

日志默认写入检测到的工作区 `log/dual_arm_control_*.jsonl`，本地优先探测仓库内
`overlay/src/functions/package.xml`，机器人端探测 `src/functions/package.xml`，不依赖测试包。无法检测时用 `--log-dir`。
日志记录完整起点、目标、原始反馈、发布、模式查询和到位结果；最多缓存 20000 个事件，
退出时写盘并记录被丢弃的旧事件数，因此不是无限时长的完整录制器。
夹爪状态、速度、开度和故障字段保留在普通原始反馈日志中（约 10 Hz），
不再生成基于静止判定变化的 `hand_hold_status` 事件。

## 旧入口迁移

以下实验入口及对应实现已移除：`aimdk_test dual_arm_control`、
`aimdk_test shoulder_direction_test`、`aimdk_test gripper_close`。
改用本文的 `aimdk_functions dual_arm_control joint/gripper`；旧固定肩抬升、
开闭夹爪和返程串联流程不再提供。
`head_test` 及 `arm_test` 的专项诊断留在测试包；后者的模式接管与交接试验不属于新命令。
历史说明与已验证事实保存在 [迁移前说明](../../../docs/archive/dual_arm_legacy_usage.md)。

新增入口需要在机器人容器内重新构建。rsync 对安装产物的保护性排除不会清除旧安装入口，
仅同步源码或 symlink 安装不能保证旧命令消失。若 `ros2 pkg executables aimdk_test`
仍列出已删除入口，应在容器确认实际安装布局及 overlay 来源后清理这些旧产物再构建，
不要继续运行旧命令，也不要猜测安装路径或改动同步排除规则。

新增脚本在 `setup.py` 注册入口，并在 `package.xml` 声明依赖。
维护者邮箱和许可证目前为占位信息，正式发布前需替换。

## 遥操第一阶段：步骤一、二开发

包内新增 `upper_body_kinematics/`。`vendor/` 成组保存参考项目的 FK、SRS IK、
批量后端和 TCP 实现；`adapter.py` 提供 `ArmKinematics`，左右分别建模。
默认 `tcp_mode='none'` 为腕原点；`hand` / `gripper` 是参考数据的名义工具点，
不能作为已完成的工具标定。custom 使用完整的腕坐标平移及旋转矩阵。
姿态矩阵将 TCP 坐标映射至 `torso_link`，位置 m、关节 rad；适配 API 不接收四元数，
后续四元数接口统一约定 xyzw。求解和观测接口均不导入 ROS。

`parameters.py` 固定包内完整官方 v1.3.0 URDF 及哈希；安装后不依赖 `underlay/extra/`
或开发者绝对路径。URDF 保留网格引用但未附网格，只用于 XML 运动学，不能直接当成
完整可视化或仿真资源包。`parameter_report()` 提供完整名称/索引/槽位/限位报告。

| 名称（分别加 `left_` / `right_`） | 单臂模型索引 | SDK 左 / 右槽位 |
| --- | --- | --- |
| `shoulder_pitch_joint` | 0 | 0 / 7 |
| `shoulder_roll_joint` | 1 | 1 / 8 |
| `shoulder_yaw_joint` | 2 | 2 / 9 |
| `elbow_joint` | 3 | 3 / 10 |
| `wrist_yaw_joint` | 4 | 4 / 11 |
| `wrist_pitch_joint` | 5 | 5 / 12 |
| `wrist_roll_joint` | 6 | 6 / 13 |

按2026-10-29用户要求，新路径的 `EXECUTION_LIMITS_RAD` 以固定参考项目
`x2ik 2.1.0` URDF限位为基准，不再与官方模型或既有命令范围取交集；
随后按用户要求，左肩roll下限降低1°、右肩roll上限提高1°，其余边界保持参考值。
数据按关节名称提取到包内 `assets/reference_joint_limits.json`，保留参考提交与URDF哈希，
在构造 SRS 求解器及批量缓存前统一注入；候选检查和完整目标组装使用同一份参数。
模型几何仍使用官方完整URDF，`parameter_report()`分别报告官方范围、原始参考范围、
调整后的执行范围及来源；`execution_limit_adjustments_deg`保留两处有符号边界调整。
旧 `dual_arm_control` 不变。反馈 FK 可观测超执行范围的
有限输入；IK 种子和候选须满足新范围，拒绝超限，不静默裁剪。

| 关节 | 左臂当前执行范围 / rad | 右臂当前执行范围 / rad |
| --- | --- | --- |
| shoulder pitch | [-3.08, 2.04] | [-3.08, 2.04] |
| shoulder roll | [-0.07845329252, 2.993] | [-2.993, 0.07845329252] |
| shoulder yaw | [-2.556, 2.556] | [-2.556, 2.556] |
| elbow | [-2.3556, 0] | [-2.3556, 0] |
| wrist yaw | [-2.556, 2.556] | [-2.556, 2.556] |
| wrist pitch | [-0.558, 0.558] | [-0.558, 0.558] |
| wrist roll | [-1.571, 0.724] | [-0.724, 1.571] |

肩roll调整使用精确的1°转rad，不使用表中显示舍入值。
最新SD反馈右肩roll为0.0645223rad，位于调整后的上限0.07845329252rad内；
该边界拒绝条件已消除；用户回传的`SD_adjusted`快照已完成双侧只读往返求解，详见下节。

`solve_initial()` 与 `track()` 分开，后者显式关闭工作空间投影和全局搜索兜底。
适配层返回的候选仍需后续验证层独立检查 FK 残差、步长及分支。

`upper_body_cartesian_core.py` 保存固定采样和单臂完整目标组装契约：
校验所选臂起点和目标，另一臂及夹爪保留采样值，不自动裁剪。
`upper_body_ros.py` 提供只读订阅、既有源时间监视和错误锁存，以及完整消息组装；
默认只记录实际模式，不强制只读观测处于 URS。这里没有运动发布器或模式切换，
消息组装不能作为运动授权。执行模式的有界服务确认、定时发布及退出状态机留至步骤六。
`upper_body_diagnostics.py` 的 `UpperBodyLog(directory, **metadata)` 按事件增量写盘，
日志目录显式指定工作区 `log/`，复用现有下载任务，不覆盖历史事件。
原始消息中的非有限浮点明确编码为字符串；日志写入失败直接报错。

依赖为 Python >=3.10 / NumPy >=1.21，面向机器人 Humble 容器。
步骤三的只读验证命令见下节；最小单臂端点运动入口见文末。
单臂平移直线离散IK及完整路径发送已实现；旋转目标与双臂同步尚未实现。
补偿依赖单独存放，不接入默认目标或发送行为。
算法与模型已完成包内适配，来源与模型差异见[第一阶段说明](../../../docs/dev_teleop_part1.md)。
步骤一、二已有用户回传的容器构建与两个合成姿态的离线FK→IK→FK结果；
后续实际反馈只读验证及运动执行结果分别记录在下节和直线入口说明中。

## 步骤三：实际反馈的只读 FK→IK→FK 验证

入口 `upper_body_kinematics_check` 只订阅手臂、夹爪与MC反馈，固定腕原点TCP
`none`，不创建运动发布器、不调用Get/Set模式服务、不提供`--execute`。
默认记录实际模式，不要求操作者为数学验证切换到URS。
要求完整14关节、两侧CLAW夹爪和MC反馈有效，沿用现有错误锁存、源时间回退/推进、
接收时效及ROS时钟检查。反馈话题/QoS沿用既有双臂命令，源码见`upper_body_ros.py`。

处理分两段：先持续处理回调并保存完整快照，再关闭ROS订阅进行数学求解。
这样不会把本程序计算期间未处理回调误判为反馈断流。计算阶段使用的是保存的历史快照，
不监视此后的现场变化。各流有独立源时间、接收时间和序号，不宣称硬件同步。

同步最新代码后，在机器人开发容器已配置的Humble/AimDK终端中构建：

```bash
cd /home/agi/aimdk-fsc
bash scripts/build_overlay.sh aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_functions upper_body_kinematics_check --help
```

固定 underlay 路径配置及首次迁移步骤见[根目录说明](../../../README.md)。首次保持当前已有稳定姿态，确保反馈节点
正常运行，无需通过本命令改变姿态或模式。先采样一次，确认订阅和输入：

```bash
ros2 run aimdk_functions upper_body_kinematics_check \
  --samples 1 --label initial_pose --log-dir ./log
echo $?
```

然后在同一姿态重复采样，观察左右结果和重复性：

```bash
ros2 run aimdk_functions upper_body_kinematics_check \
  --samples 5 --interval 1 --label pose_A --log-dir ./log
```

`--samples`默认3、`--interval`默认1秒；重复静止姿态采样不代表已覆盖不同姿态。
待操作者按既有现场流程形成其他已允许的姿态并稳定后，再分别以`pose_B`、`pose_C`
等标签重复。这个命令不会帮助移动到其他姿态，不要求摆到限位附近。
可用`--side left/right`只计算一侧，但采样仍保留并校验完整双臂、夹爪及MC反馈。
硬件实际零位、轴向和TCP定位精度仍需后续独立测量。

按实际已核实的信息补充`--sdk-version`、`--firmware-version`、`--load-description`；
未提供时明确记录为`unknown`。`--timeout`默认5秒，分别用于启动及等待下一快照；
`--feedback-timeout`默认0.5秒，允许0.2..1秒，与既有命令范围一致。反馈错误、
源时间冻结/回退或超时按原契约报错，不通过重新打接收时间或裁剪输入继续。

在任一已能正常采样的姿态，可额外运行无发布的数学用例：

```bash
ros2 run aimdk_functions upper_body_kinematics_check \
  --samples 1 --check-cases --label boundary_math --log-dir ./log
echo $?
```

`--check-cases`在采样结束后检查两侧每关节的精确限位、相邻内侧浮点数、
相邻外侧种子及关节目标，另含非有限种子和明确不可达的空间目标。
完整双侧共116项用例；所有目标只在内存中计算，不发送到机器人，不要求实机处于这些姿态。
边界FK→IK的残差、无解或候选拒绝仅作观测，不任意新增残差门槛；
越界种子和关节目标必须明确拒绝，不可达目标应返回无解或拒绝，意外返回候选会报告异常。
合成边界用例采用范围中点作为其他关节输入，不是实际快照或运动计划。

每次生成独立的`log/upper_body_*.jsonl`，启动时打印准确文件名，逐事件写盘。
`metadata`包含代码摘要、实际安装路径、模型哈希、完整限位、环境和实验标签；
`feedback`保留完整原始消息及错误，`snapshot`关联各流独立源时间/接收时间/序号；
`round_trip`包含请求位姿、候选、独立回算残差、原始及绕角种子距离、SEW、分支和计算耗时；
`diagnostic_case`包含合成用例及预期匹配情况，`summary`记录数量，`exit/error`保留退出原因。
耗时包含调用及诊断计算，不等于连续控制周期性能。非有限原始值以明确字符串保存。

退出码`0`表示采样完成、实际快照均返回有限且符合限位的候选、所启用的失败用例符合契约；
它没有施加实测精度门槛，不代表运动授权或定位验收。`2`表示实际快照无解/超限拒绝等不完整
结果或用例契约异常；`1`表示环境、反馈、文件或运行异常；`130`表示用户中断。
实际关节超出新执行范围时仍保存FK与超限列表，但拒绝以该种子求解，不能改范围后算作通过。
中途失败或中断时保留已写入的反馈及快照，可能没有最终summary。

程序退出后，复用“机器人：下载日志”任务取回完整JSONL和终端输出。
审阅时分别检查：输入是否完整新鲜、实际模式、超限原因、左右残差/种子距离/分支、
失败用例是否符合预期。重复采样、多个姿态和完整用例用于后续覆盖，不作为首次最小闭环的必做流程。
用户已回传肩roll调整后的`SD_adjusted`日志：双侧实际快照均返回限位内候选，
异常计数0，模型内往返残差约为浮点计算精度；尚未验证实体TCP定位精度。

## 最小单臂端点运动闭环

`upper_body_move`从一次实际快照出发，在`torso_link`下沿x/y/z轴给定平移增量，
固定起点旋转、腕原点TCP、另一臂和双侧夹爪的采样目标。
零位移直接保留原关节角；非零位移用当前关节角和SEW臂角调用局部`track()`，
关闭工作空间投影和全局搜索兜底，避免全局初解的限位偏好导致约21°的姿态重排。
首次局部求解不预设解析分支，记录实际返回分支；无解或超限明确拒绝，不重试或放宽姿态。

本轮只做一个笛卡尔端点的IK，然后通过原生`/mc/upper_body_command`做一次
MoveJ式关节五次插值。它不是参考项目`Robot.moveJ()`的直接调用，
中间TCP路径也不保证为笛卡尔直线。直线离散IK跟踪见下方`upper_body_line`。
所选臂每关节总变化沿用既有单次实验的10°上限，时长沿用5..10秒；
该上限用于本次小幅验证，不是机器人额定运动能力。独立回算位姿残差写入日志及终端，
不额外引入未经实测的精度门槛。

用户同步后在机器人Humble/AimDK容器构建并加载：

```bash
cd /home/agi/aimdk-fsc
bash scripts/build_overlay.sh aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_functions upper_body_move --help
```

最小验证只需两次调用：先在当前有效姿态只读查看一个小端点，例如左臂沿torso的+x方向5mm：

```bash
ros2 run aimdk_functions upper_body_move \
  --side left --axis x --delta-mm 5 --log-dir ./log
```

核对打印的实际起点、端点、全部关节变化和FK残差。若返回无解/超限，先回传日志，
不增加步长或改限位绕过。可选用`--delta-mm 0`检查保持当前姿态的分支，不是必做测试。
只读调用不要求URS，不创建运动发布器或模式客户端。

确认现场动作空间及没有其他上肢目标发布者后，由操作者按已有流程进入URS/RUNNING，
再执行一次：

```bash
ros2 run aimdk_functions upper_body_move \
  --side left --axis x --delta-mm 5 --duration 5 --execute --log-dir ./log
```

执行调用会重新采样，以本次姿态规划相对位移；不会重用上一次只读调用的绝对目标。
执行前有界查询GetMcAction确认URS/RUNNING，运行期间继续检查模式和反馈。
沿用50Hz发送、80ms发布间隔上限、全14关节±3°跟踪检查、终点新源反馈连续确认0.3秒、
关节轨迹结束后额外等待最多2秒；夹爪开度误差只记录。
本轮按操作者要求屏蔽本命令的夹爪状态/静止门槛：采样、发布、执行循环及终点确认
均不再以`state`或静止候选阻断，速度也不作为静止门槛；仍保持采样开度，
保留完整夹爪原始反馈、故障码、有限值、类型/数量及源时间/时效检查。
终端和日志明确记录`hand_state_check=False`；`dual_arm_control`也已按操作者要求屏蔽夹爪状态及静止门槛，见前述说明。
夹爪状态来自`/aima/hal/joint/hand/state`的`aimdk_msgs/msg/HandStateArray`，
读取`left_hands[0].state`与`right_hands[0].state`；SDK说明为
0到达目标、1移动中、2堵转、3物体掉落。
增加`--delta-mm`时仍先只读核对计划；逐关节10°总变化上限与有效限位不变。
发布前按当前新遥操限位再检查所选臂目标，未选臂保持采样值、不裁剪。
有限且残差小的数学候选不等于实体TCP精度验收；终点±3°确认也不证明精确移动了5mm。

正常完成、失败或Ctrl+C都停止本命令发布，不自动返程或切换模式；
停止发布不是硬件急停，也不保证退出后固件持续保持。
日志增量写入`log/upper_body_*.jsonl`，包含起点、端点计划、完整发送目标和原始反馈，
结束报告实测关节变化及反馈FK；等待退出后通过现有日志任务下载。
首次先验证一侧即可，不需要重复跑116项数学用例。

用户已在机器人环境回传左臂10mm、15mm端点运动执行记录，
每次发布265帧并以`completed`结束；此前5mm尝试曾因夹爪静止门槛在发布前中止。
这些记录验证端点规划及关节插值执行链路，不代表实体TCP定位精度验收。


### 单臂直线路径：upper_body_line

`upper_body_line`沿用一次实际采样、腕原点TCP、torso坐标轴和固定起点姿态。
区别于`upper_body_move`仅求终点，本命令先用五次时间曲线生成整条直线位置序列，
时间网格不大于20ms（5秒为251点），逐点调用局部`track()`。
首点直接保留原角，首个非零点确定解析分支，后续以前一点关节、SEW及分支连续求解。
未启用补偿时零位移整段保持原角；不进行全局搜索、投影或姿态放宽。
显式位置偏置补偿用法见下节。
整条路径接受后才允许创建发布器；无解、越限或分支改变时报告失败点并拒绝执行。
逐点相对采样起点及相邻关节变化沿用10°上限，不新增未经实测的精度门槛。

运行时按实际经过时间取路径目标，以50Hz发布；相邻网格点之间采用线性关节插值，
因此这是离散直线规划的近似执行，不宣称连续实体TCP严格为直线。
运行期复用原有URS/RUNNING、反馈时效、±3°跟踪和终点连续0.3秒确认。
**夹爪状态及静止检查继续屏蔽**，仍保持采样开度，保留故障码及反馈有效性检查。
未选臂保持本次采样目标，不自动返程或换模式。
已观察到URS首次发布时双臂约0.5秒内的偏移，默认只记录；显式补偿只作用于所选臂，
不跳过对应反馈检查；到位成功不等于毫米级轨迹精度验收。

同步后在机器人Humble/AimDK容器内构建：

```bash
bash scripts/build_overlay.sh aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
```

先只读查看左臂torso +X 10mm、5秒的完整规划：

```bash
ros2 run aimdk_functions upper_body_line \
  --side left --axis x --delta-mm 10 --duration 5 --log-dir ./log
```

核对关节变化、整条路径最大位置/姿态残差和最大相邻关节变化后，
由操作者进入URS/RUNNING，再执行一次（会重新采样和规划）：

```bash
ros2 run aimdk_functions upper_body_line \
  --side left --axis x --delta-mm 10 --duration 5 --execute --log-dir ./log
```

日志仍为`log/upper_body_*.jsonl`，元数据`command=upper_body_line`。
`line_plan_point`保存每点时间、请求位置、FK位置、完整关节目标、残差、SEW和分支；
`plan`保存最大残差和变化摘要；`motion_begin`保存执行单调时钟起点，
结合`command_published`及原始`feedback`可按时间回算轨迹，分别观察启动偏移和后续误差。
等待退出后复用现有“机器人：下载日志”任务，无需重复整套数学用例。
用户已确认连续局部IK、完整路径发送、规划后刷新及反馈记录闭环跑通。
实体直线精度及姿态保持继续单独研究，±3°到位不能替代实体精度验收。


直线入口首次实机尝试（`1793278865292187468_21af3c50`、
`1793278949661644607_51b81220`）均完成251点规划，但未发布任何命令。
规划耗时约0.70/0.79秒，规划后的首条MC回调读到积压消息，源时间年龄为
0.632/0.724秒，触发既有0.5秒时限。此处已修正刷新条件：除接收时间外，
各路源时间也必须晚于规划后刷新开始的ROS时间，才进行完整时效检查。
等待仍受`--timeout`限制，期间持续解码、保留回退/故障等错误锁存，不扩大
反馈时限或改写源时间。日志增加`capture_refresh_begin`及超时快照。
该刷新修正已由用户复测，闭环跑通。


### 步骤七：显式位置偏置补偿

`upper_body_line`和`upper_body_move`均增加`--compensation`，默认关闭。
本次为**官方X2 Ultra v1.3.0模型的适配版**，不是参考模型原版效果结论。
使用已提取的`ArmDynamics.gravity_torque(q_ideal)`，
`raw_bias=τg/40`，`bounded_bias=clip(raw_bias, ±12°)`；固定参数不调优。
为从采样关节目标平滑接入，发送偏置为`a(t)·bounded_bias`，
`a(t)`沿用本次运动时长的五次进度，终点及到位等待期间为完整偏置。
因此显式补偿下的零位移也会引入偏置，不再是保持原角。
原始IK路径、SEW及分支状态不受偏置反向修改。

重力输入固定使用`/aima/hal/imu/torso/state`的骨盆四元数和
`/aima/hal/joint/waist/state`的三腰角（按名称映射）。
`R_world_torso=R_world_pelvis·R_pelvis_torso(q_waist)`，
`g_torso=R_world_torso.T·[0,0,-9.81]`。
模型骨盆IMU安装旋转为单位阵；实机观测该话题`frame_id=base_link`，
模型中base_link与pelvis重合；也接受`pelvis`及`imu_in_pelvis_link`标签。
不根据`base_link`标签把chest与pelvis话题等价处理。
运行时拒绝无姿态标记、零/非有限四元数、无效源时间及未知标签；
源时间推进、回退、源年龄、接收时效沿用既有检查和错误锁存。
不采用上游低通/倾角拒绝后保留旧值的行为，不使用加速度计或静态重力兜底。
IMU/腰源时间差记录为观测量，不宣称硬件同步，也不新增时间差阻断阈值。

只修改所选臂，未选臂和两夹爪继续固定重发采样值。
最终候选检查当前执行范围，超限拒绝，不采用上游的再次位置裁剪。
原IK路径的10°变化约束保留；额外补偿单独受±12°偏置限幅及最终位置限位检查。
**±3°跟踪和到位条件均保留，比较对象为补偿后的发送目标**；
终点还要求完整偏置已经实际发布。相对原IK目标的误差另外记录，不能用到位结果
宣称物理末端精度或补偿有效。超过既有跟踪容差仍会中止，不为补偿扩大容差。

附加载荷默认0kg，即只计算URDF中的七个臂连杆；没有自动识别夹爪、工具或工件质量。
如有已知附加载荷，用`--payload-mass-kg`和`--payload-com-m X Y Z`提供；
质心相对腕原点，在所选侧wrist_roll_link坐标中表达，单位m。
当前运动入口TCP固定none，不将名义TCP中心自动当成负载质心。
`--load-description`用于保存实际姿态、工具与负载条件，未知项明确写unknown。

复用既有同步任务后，在机器人Humble/AimDK容器构建：

```bash
bash scripts/build_overlay.sh aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
```

先只读查看与前轮相同的小范围动作；以下示例只计算URDF臂连杆重力，
实际侧别、方向、幅度和负载参数按现场上一轮条件填写：

```bash
ros2 run aimdk_functions upper_body_line \
  --side left --axis x --delta-mm 10 --duration 5 \
  --compensation --log-dir ./log \
  --load-description '记录当前姿态、夹爪、工具和负载；本次附加载荷模型为0kg'
```

只读阶段保存整条原IK路径，并用采样重力逐点预览补偿后路径；
任何补偿后候选超限均在创建发布器前拒绝。
核对日志和终端偏置后，由操作者进入URS/RUNNING，再显式执行同一配置：

```bash
ros2 run aimdk_functions upper_body_line \
  --side left --axis x --delta-mm 10 --duration 5 \
  --compensation --execute --log-dir ./log \
  --load-description '记录本次实际姿态、夹爪、工具和负载'
```

执行前刷新包含IMU与腰的全部反馈；执行时逐周期使用新鲜且有效的最新样本重新计算，
发布前再次核查时效、跟踪和位置范围。预览快照不直接作为执行期重力缓存。
对照实验省略`--compensation`，分别保存起始姿态及负载条件；程序不自动返程。

日志新增`compensation_config`和`compensation_plan_point`，记录模型哈希、固定参数、
负载、适配差异、原IK目标、躯干重力、估计力矩、原始/限幅/实际引入偏置、
限幅标记、输入源时间/接收时间/时间差及最终目标。
`command_published.compensation`记录该条实际发送命令对应的同组诊断；
`execution_result`同时保存原IK目标、最后发送值及反馈FK，原始IMU/腰消息也增量保存。
复用“机器人：下载日志”任务回传。用户已回传左臂torso +X 10mm、20mm各5秒的
有补偿执行记录（`1793279843957513668_25601ed8`、`1793279862525478573_cac3cd4d`），
两次均发布265帧并完成既有终点确认。与此前无补偿记录比较，反馈关节经模型FK回算的
终点误差分别由14.85/15.11mm降至3.68/5.35mm。
对照起始姿态不同，各配置各幅度仅一次执行；该观测不等于受控实验的通用收益或实体TCP精度。
启动偏移仍存在，补偿继续作为默认关闭的显式实验选项。
