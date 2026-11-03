# 自定义功能工具

适用：X2 Ultra、官方 v1.3.0 模型、机器人 Ubuntu 22.04 / ROS 2 Humble / AimDK 环境。
ROS 2 包名为 `aimdk_functions`，使用 `ament_python`，依赖 NumPy >=1.21。

运动统一入口为 `upper_body_control joint/tcp/gripper`。保留 `volume_control`、
`arm_compensation_probe` 和 `upper_body_kinematics_check` 独立工具。
整合后的统一入口已由用户完成机器人端验证；覆盖范围见[启动保持调查](../../../docs/archive/upper_body_startup_hold_investigation_20261030.md)。
本轮冗余模块整理仅完成本地静态、语法检查，需同步后重新构建并验证。
历史命令、参数和实机结果见[整理前说明](../../../docs/archive/upper_body_commands_before_unification_20261030.md)
与[上肢验证记录](../../../docs/archive/x2_upper_body_validation_20261029.md)。

## 构建与入口

用户通过现有任务同步后，在机器人 Humble 容器工作区根目录执行：

```bash
bash scripts/build_overlay.sh aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_functions upper_body_control --help
ros2 run aimdk_functions upper_body_control tcp --help
```

固定 underlay 为 `/home/agi/aimdk/install`；其他安装位置使用 `AIMDK_UNDERLAY_INSTALL`，
并加载对应环境。流程见[项目说明](../../../README.md)。

旧 `dual_arm_control`、`upper_body_move`、`upper_body_line` 入口及其执行实现已移除。
分别改用 `upper_body_control joint/gripper`、`tcp --path endpoint`、`tcp`。
原 `upper_body_move_core.py`、`upper_body_line_core.py` 的规划实现已合并至
`upper_body_cartesian_core.py`；共用日志目录探测与代码摘要函数归入 `upper_body_diagnostics.py`。
同步排除会保护旧安装产物；新增入口必须重新构建，旧可执行文件可能仍出现在
`ros2 pkg executables aimdk_functions`。确认实际安装布局和 overlay 来源后清理旧产物，
不要继续运行旧入口。保持现有同步排除规则。

## 控制约定

所有动作默认只读，仅显式 `--execute` 创建发布器。执行前由操作者进入
`UPPERBODY_REMOTE_SPLIT`（URS）并等待 `RUNNING`，确认动作空间及无其他节点同时发布上肢目标。
启动时有界查询 GetMcAction，执行期间持续检查原生模式反馈。
只读规划允许其他模式；规划后刷新反馈，再检查起点漂移与时效后开始执行。

每次发送 `/mc/upper_body_command` 完整左7右7关节 rad 目标和模式1的左右夹爪开度。
手臂按名称映射。未选关节保持本次采样的规划起点，夹爪保持采样实际开度；
开启补偿时所选臂的七个关节均可能改变。
正常结束、失败、中断均停止发布并退出，不自动返程或切换模式。
停止发布不是急停，退出后的持续保持与固件超时行为待实机确认。
夹爪开度为位置目标，不提供力保持保证。

| 子命令 | 目标 | 默认行为 |
| --- | --- | --- |
| `joint` | 同名单关节带符号增量 | 单程5秒，补偿开启 |
| `tcp` | torso_link系XYZ相对平移，保持起始姿态 | 直线路径，单程5秒，补偿开启 |
| `gripper` | 0..1开度，0闭合、1全开 | 按timeout持续发送，双臂保持采样值 |

三类动作均要求 `--side left/right/both`。双侧共用目标与独立目标互斥；
独立目标仅在both下接受，必须同时给出左右两侧。两侧共用时长，独立输入不自动镜像。

### 手臂规划起点

适用X2 Ultra、SDK 1.1原生上肢入口。当前实现默认
`--arm-start-source mc-command`：订阅MC发布的`/aima/hal/joint/arm/command`
（`JointCommandArray`），以完整14关节的`position`按名称重排为左7右7 rad，
初始化关节轨迹、TCP的FK/IK参考及夹爪动作期间的双臂保持目标。
`--arm-start-source state`使用`/aima/hal/joint/arm/state`实际位置作对照。
命令来源缺失、位置无效或超时均失败退出，保持所选来源。

命令起点在规划时冻结，首帧原生`arm_pos`严格取该起点，补偿进度为0；
随后按原时间轴推进轨迹，补偿沿用原算法及默认值。HAL命令在执行中继续只读记录，
轨迹不跟随HAL命令重新定基。实际关节反馈仍用于故障、时效、漂移、跟踪及终点检查。
原生接口仅发送位置，HAL的velocity、effort、stiffness、damping保留为日志观测。
命令参考与反馈的同零点映射、首帧下沉改善及补偿叠加效果均待真机验证；
依据见[命令观测报告](../../../exports/upper_body_command_watch_20261102/report.md)。

首次验证在URS/RUNNING下执行零增量、关闭补偿，观察开始发布时的实体姿态及日志：

```bash
# 先只读核对起点；确认后以相同参数增加 --execute。
ros2 run aimdk_functions upper_body_control joint --side both \
  --tag shoulder_pitch --delta-deg 0 --no-compensation
```

检查`arm_start_reference`与首条`command_published`的14关节位置一致；
比较启动前后`feedback`中的`arm`实际位置及`arm_command`完整字段，记录现场是否下沉。
再由操作者恢复相同初始条件，用`--arm-start-source state`重复对照。
零增量衔接验证后，再验证小幅joint/TCP动作及补偿开启行为。

## 关节增量

`--tag` 支持 `shoulder_pitch`、`shoulder_roll`、`shoulder_yaw`、`elbow`、
`wrist_yaw`、`wrist_pitch`、`wrist_roll`，对应每侧索引0..6，SDK左侧槽位0..6、右侧7..13。
每个所选关节增量绝对值不超过10°；`--duration`为5..10秒，默认5秒。
起终点速度和加速度为零的五次关节插值。0增量允许，开启补偿时仍会引入位置偏置。

```bash
# 只读：左腕roll增加2°，默认开启补偿。
ros2 run aimdk_functions upper_body_control joint --side left --tag wrist_roll --delta-deg 2

# 双侧同名关节使用独立增量，关闭补偿；显式执行。
ros2 run aimdk_functions upper_body_control joint --side both --tag wrist_yaw \
  --left-delta-deg 2 --right-delta-deg -2 --no-compensation --execute
```

## TCP路径与双臂调度

TCP暂按法兰面中心等同 `{side}_wrist_roll_link` 原点，`tcp_mode=none`。
位置计算单位m，CLI位移单位mm，关节rad；姿态矩阵将TCP坐标映射至 `torso_link`。
本入口保持每侧采样姿态，暂不提供姿态增量或绝对位姿目标。

默认 `--path line`：按20ms统一时间网格和五次进度生成直线点，左右分别以上一点
关节/SEW作局部IK，首个非零点确定解析分支后锁定。5秒轨迹为251点。
全部点规划成功后才进入执行，执行时在相邻关节点之间插值，点间TCP路径为近似直线。
`--path endpoint`：对平移端点求一次局部IK，再做关节五次插值，中间TCP路径由关节轨迹决定。

每个IK点相对采样起点的单关节变化不超过既有10°实验范围；直线相邻点也沿用10°约束。
零位移保留采样关节。无解、非有限、越界或分支改变则拒绝；不投影、不全局搜索兜底、不裁剪。
记录FK残差，不新增未经实测的残差阻断阈值。

```bash
# 单轴写法，默认直线、默认补偿；先只读。
ros2 run aimdk_functions upper_body_control tcp --side left --axis x --delta-mm 10

# XYZ向量写法，显式端点模式并执行。
ros2 run aimdk_functions upper_body_control tcp --side right \
  --translation-mm 10 0 0 --path endpoint --execute

# 双臂分别沿torso Y反向移动5mm，共同时长5秒，默认补偿。
ros2 run aimdk_functions upper_body_control tcp --side both \
  --left-translation-mm 0 5 0 --right-translation-mm 0 -5 0 --duration 5

# 双臂使用相同torso系位移。
ros2 run aimdk_functions upper_body_control tcp --side both --translation-mm 10 0 0
```

`--axis`只与`--delta-mm`配合，省略axis时为x；单轴与XYZ写法互斥。
双侧独立位移使用 `--left-translation-mm X Y Z` 与 `--right-translation-mm X Y Z`。

调度器基于同一次完整采样预规划两侧轨迹，执行时共用一个单调时间进度和50Hz发布定时器。
每周期取两侧理想目标，用同一组最新有效IMU/腰反馈分别计算补偿，统一组装并校验完整目标后发布。
任一侧失败均拒绝整次规划或中止整次发布。同步指共用时间轴和同帧提交；实体双臂同步误差待测。
当前提供单次轨迹，不包含在线遥操IK或双臂相对位姿约束。

## 位置偏置补偿与负载

`joint/tcp`默认启用，`--no-compensation`关闭，`--compensation`显式开启。
纯 `gripper`不接受补偿或负载参数。单侧控制只补偿所选臂；双侧分别计算后合并，
未选臂和夹爪保持采样值。关节命令的“单关节”指理想轨迹，补偿可改变所选臂全部关节。

固定参数为 `Δq=clip(τg/40, ±12°)`，刚度40 N·m/rad，偏置乘本次运动五次进度从零引入。
使用官方完整v1.3.0惯性模型与当前理想关节位置估算重力力矩，不使用effort估力。
额外偏置与原轨迹10°范围分别约束；补偿后的最终位置越限明确拒绝。
启用补偿后理想TCP路径仍按请求规划，发送目标与反馈FK单独记录，直线请求不构成实体精度保证。

重力输入为 `/aima/hal/imu/torso/state` 的骨盆IMU姿态及
`/aima/hal/joint/waist/state` 的具名三腰关节。接受已核对标签 `base_link`、`pelvis`、
`imu_in_pelvis_link`，无姿态、未知标签或无效源时间拒绝；不使用静态重力或旧缓存兜底。
IMU/腰源时间差记录为观测量，未增加时间差阻断阈值。规划使用采样输入逐点预览，
执行前刷新全部反馈，执行中每次发布使用当前有效样本重算。

附加载荷默认0kg，仅计算模型七个臂连杆；夹爪、工具、工件质量不自动加入。
`--payload-mass-kg`与`--payload-com-m X Y Z`提供共用负载，质心在各侧腕坐标系中表达，单位m。
双臂分别使用 `--left-payload-mass-kg`、`--left-payload-com-m` 及对应right参数覆盖共用配置。
禁止指定未选侧负载；关闭补偿时不接受负载配置。TCP与负载质心独立配置。

```bash
ros2 run aimdk_functions upper_body_control tcp --side both --translation-mm 10 0 0 \
  --left-payload-mass-kg 0.2 --left-payload-com-m 0 0 0.05 \
  --right-payload-mass-kg 0 --load-description '记录实际姿态、工具和负载条件'
```

上述负载仅为参数示例，现场填写实际已知值。现有补偿收益证据限于历史左臂小幅动作，
右臂、双臂和关节控制的补偿效果待独立验证。

## 统一关节限位

执行范围唯一来源为 `upper_body_kinematics/parameters.py` 的 `EXECUTION_LIMITS_RAD`：
参考项目x2ik 2.1.0范围，左肩roll下限减1°、右肩roll上限加1°。
此约定写在启动脚本及参数模块开头。关节、IK、补偿后的目标共用该范围，不与旧SDK范围取交集。
固定来源和哈希保留在 `assets/reference_joint_limits.json`，`parameter_report()`报告官方范围、
原始参考范围、调整后范围及两处有符号调整。几何与惯性仍使用包内官方v1.3.0模型。

| 关节 | 左臂范围/rad | 右臂范围/rad |
| --- | --- | --- |
| shoulder pitch | [-3.08, 2.04] | 相同 |
| shoulder roll | [-0.07845329252, 2.993] | [-2.993, 0.07845329252] |
| shoulder yaw | [-2.556, 2.556] | 相同 |
| elbow | [-2.3556, 0] | 相同 |
| wrist yaw | [-2.556, 2.556] | 相同 |
| wrist pitch | [-0.558, 0.558] | 相同 |
| wrist roll | [-1.571, 0.724] | [-0.724, 1.571] |

肩roll调整按精确1°转rad计算。关节关闭补偿时检查所选关节起点和目标；TCP或开启补偿时
检查所选臂全部关节。未选目标原样保持，不自动裁剪；范围检查不包含碰撞检查。

## 夹爪控制

共用 `--opening` 或双侧独立 `--left-opening`、`--right-opening`，均为有限0..1。
夹爪目标在首帧更新，按 `--timeout`持续发送，默认5秒、范围1..10秒，不生成手臂运动轨迹。

```bash
ros2 run aimdk_functions upper_body_control gripper --side right --opening 0.5
ros2 run aimdk_functions upper_body_control gripper --side both --opening 1 --execute
ros2 run aimdk_functions upper_body_control gripper --side both \
  --left-opening 1 --right-opening 0 --execute
```

沿用此前确认：夹爪state及速度不作起点、跟踪或到位门槛，非零故障码仍中止。
开度、状态、速度、故障保留日志，不设置夹爪开度误差容差。
成功退出表示命令持续发送与双臂保持检查完成，夹爪是否达到目标需查看反馈与实物。

## 反馈、结束条件与日志

保留源时间推进/回退、源年龄、接收时效、ROS时钟、硬件故障及首次错误锁存检查。
`--feedback-timeout`统一设置各路接收、源停滞与源年龄时限，默认0.5秒，范围0.2..1秒；
ROS时钟停滞上限沿用0.2秒。仅命令使用当前ROS时间，不改写反馈源时间。
反馈QoS沿用best_effort/transient_local，补偿传感器及HAL命令best_effort/volatile，发布深度10。

发布50Hz，间隔上限80ms；全部关节跟踪容差±5°，终点容差±5°，终点新反馈连续确认0.3秒，
轨迹或夹爪持续发送结束后额外最多等待2秒。终点须已实际发布完整目标，补偿进度达到1。
到位比较对象为实际发送目标，原理想轨迹和反馈FK另行记录。
执行前以采样实际反馈为基准检查姿态漂移，容差保持±3°；mc-command来源还以采样命令
为基准复核命令漂移，容差同为±3°。命令与实际位置的静态差值记录在日志，
执行时仍受上述±5°跟踪及终点约束。日志分别记录跟踪与终点容差。
不超过5°的动作可能在实际运动很小时满足确认，不能据此认定精确执行了输入增量。
`--timeout`还用于启动反馈、服务发现/响应与DDS匹配的各自有界等待。

日志默认在自动探测的工作区 `log/upper_body_*.jsonl`，失败时用 `--log-dir`明确指定。
ROS节点创建后立即建立反馈订阅，先于模式查询和运动学模型初始化；收到反馈即写入日志。
`feedback_subscription_ready`标记订阅就绪，首条`feedback`标记实际开始收到数据。
首次接收仍取决于ROS初始化与DDS发现；规划计算期间按现有单线程回调处理，不保证连续无丢帧采集。
按事件增量写入并flush，包含完整原始反馈与时间、代码/模型哈希、限位、请求、分侧IK/SEW/分支、
路径残差、补偿参数/偏置/源时间、理想与实际发送目标、周期耗时、反馈FK及退出原因。
`arm_start_reference`记录起点来源、命令与实际位置差及各自源时间和源时间差；
命令与反馈采用各自最新有效样本，未作硬件同步。`state`对照只订阅原有反馈流，
需要同时观察HAL命令时使用实验包的`upper_body_command_watch`。
未知SDK/固件版本标记unknown，真实版本由操作者核对。日志复用现有“机器人：下载日志”任务。

## 实现结构与独立观测工具

| 模块 | 职责 |
| --- | --- |
| `upper_body_control.py` | 参数、单次生命周期与日志 |
| `upper_body_control_core.py` | 分侧目标、统一轨迹、双臂合并与补偿组装 |
| `upper_body_execution.py` / `upper_body_mode.py` | 单发布器、时间调度、终点确认／有界模式查询 |
| `upper_body_cartesian_core.py` | 完整目标契约、TCP端点与直线纯IK规划 |
| `upper_body_ros.py` / `dual_arm_control_core.py` | ROS反馈适配／纯解码、时效、错误锁存、关节插值 |
| `upper_body_compensation.py` | 分侧位置偏置计算 |
| `upper_body_diagnostics.py` | 共用增量日志、工作区日志目录探测及代码摘要 |
| `upper_body_acceptance.py` / `upper_body_acceptance_replay.py` | 交互验收记录／实际快照离线回放 |

包内模型资源独立安装，不依赖 `underlay/extra`。只读观测命令保持原行为：

```bash
ros2 run aimdk_functions upper_body_kinematics_check --side both --log-dir ./log
# 操作者预先进入相应模式，tag只指定预期，不切换模式。
ros2 run aimdk_functions arm_compensation_probe --tag SD
ros2 run aimdk_functions arm_compensation_probe --tag URS
```

probe保存metadata、图快照、原始events、summary与report；其退出码0表示观测完整且模式匹配，
不作为补偿有效性结论。详细字段和历史记录见[整理前说明](../../../docs/archive/upper_body_commands_before_unification_20261030.md)。

## 机器人端验收与待办

### 交互式验收记录

`upper_body_acceptance`用于收集[遥操第一阶段](../../../docs/dev_teleop_part1.md)前五步的证据。
新增入口后须在机器人Humble容器内重新构建一次：

```bash
bash scripts/build_overlay.sh aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_functions upper_body_acceptance
```

脚本逐项询问：`p`执行只读预览，`s`或默认输入跳过；预览成功后输入完整的`EXECUTE`
执行一次单程动作。每次执行独立确认，不自动返程。正反向请求均相对当次采样起点，
不构成位置恢复。先退出正在运行的控制命令再同步代码。
运动前由操作者进入URS/RUNNING并确认动作空间和发布者；具体约定沿用上文。

| 步骤 | 脚本收集内容 |
| --- | --- |
| 1 | 安装包路径、代码/模型哈希、来源及限位报告，子命令help，参数非法值/混用/缺失/超限拒绝 |
| 2 | 左右各三次实际快照FK→IK→FK，数学边界、越界和不可达用例；多姿态可分会话采集 |
| 3 | 从保存快照离线生成左右平移、旋转、组合轨迹，每条5秒251点，逐帧记录及统计耗时、步长、残差 |
| 4 | 左右七关节正反向、三个轴正反向直线、endpoint、夹爪、人工中断；补充双臂共用/独立关节、独立TCP及夹爪目标 |
| 5 | 左右各三组补偿开关对照，记录实际负载、起点差异和现场测量；操作者每次手动恢复起点 |

默认关节幅度2°、平移5mm；离线回放旋转1°绕初始TCP局部Z，平移沿torso X，
组合轨迹共用五次进度。幅度可用`--joint-deg`、`--translation-mm`、`--rotation-deg`
设置，配对次数用`--repeats`设置。较小动作仍需独立测量实际增量。
纯旋转及完整位姿实机请求尚无统一入口，步骤4保留未完成；双臂独立负载和故障/断流注入需另行补充。

按阶段运行或使用已有实际快照回放：

```bash
ros2 run aimdk_functions upper_body_acceptance --steps 1 2 3
ros2 run aimdk_functions upper_body_acceptance --steps 4 --joint-deg 2 --translation-mm 5
ros2 run aimdk_functions upper_body_acceptance --steps 5 --repeats 3
ros2 run aimdk_functions upper_body_acceptance --steps 3 --snapshot-log /实际路径/upper_body_快照.jsonl
```

默认输出至工作区`log/upper_body_acceptance_<时间>_<随机标识>/`，可用`--log-dir`指定父目录。
现有“机器人：下载日志”任务可下载整个会话。目录包含`report.md`、`report.json`、
逐用例终端输出、控制器原始JSONL和离线逐帧JSONL。每项完成即保存，中断保留已有证据；
可在新会话中用`--steps`补充验证，最终审阅同时引用各会话目录。

运行中可在另一终端查看用例目录的`console.txt`。执行时Ctrl+C转发给控制命令并保留中断记录，
交互提示时Ctrl+D结束会话。日志的`commands_stopped`表示本脚本停发，实体保持及固件超时表现由现场记录。

`report.json`保存各关节实际增量、发送目标/理想目标终点误差、接收时跟踪误差、
发布间隔与计算耗时分布、补偿偏置及输入源时间差、反馈FK和原始错误。
跟踪误差采用反馈接收前最近的已发送目标，仅作离线观测。执行结果分别记录自动证据与
人工`pass/fail/pending`、独立测量及现场观察；分步审阅使用`accepted/incomplete/failed`。
缺少完整证据时不能记录为accepted，步骤4因缺少旋转/完整位姿入口固定保留incomplete。
会话退出码0表示报告正常保存，阶段验收由覆盖范围、现场测量和补偿适用结论共同评定。

### 其他验收项目

先按同一配置只读预览，再显式执行；记录实际姿态和负载。

| 操作 | 预期结果 |
| --- | --- |
| 构建后查看三个子命令help | 新入口可用，默认TCP直线、手臂补偿开启，gripper拒绝补偿参数 |
| 左右分别joint，开启/关闭补偿 | 槽位与增量正确；理想轨迹仅改变指定关节，补偿仅作用于选臂 |
| 单臂tcp直线及显式endpoint | 分支、路径与终点记录完整，另一侧保持采样目标 |
| 双臂独立TCP目标及独立负载 | 两侧共用时间进度、同帧发送，补偿结果无互相覆盖 |
| 单爪及双爪独立开度 | 首帧更新所选开度，双臂保持，不因state/速度中止，故障仍中止 |
| 非有限/缺少一侧/混用参数/超限输入 | 规划拒绝且未发布动作 |
| 反馈故障、正常结束与人工中断 | 错误锁存，统一停发，无自动返程或模式切换 |

待办：独立验证实体法兰面中心与腕原点的平移/方向关系；验证双臂调度周期、右臂/双臂补偿收益、
实体直线精度与退出后的保持行为。当前配置不包含实体TCP标定结论。

## PICO双臂只读回放

`remote_control`负责XR输入、Grip会话、坐标映射及输入日志；本包负责X2双侧逐帧IK。
`upper_body_teleop_replay`从XR日志和机器人命令／反馈快照回放，不导入ROS、不发布运动、补偿关闭，
`--execute`明确拒绝。机器人容器内先构建`remote_control`与`aimdk_functions`，再运行：

```bash
ros2 run aimdk_functions upper_body_teleop_replay \
  --xr-log /实际路径/xr_input.jsonl \
  --mapping-config /实际路径/mapping.json \
  --snapshot-log /实际路径/upper_body_快照.jsonl \
  --snapshot-index 0 --load-description '实际工具和负载'
```

映射配置及XR日志格式见[remote_control说明](../remote_control/README.md)。
快照使用`upper_body_kinematics_check --include-arm-command`生成的`snapshot`事件，
或现场只读遥操日志中的`snapshot`；关节按名称解码为左7右7。
默认`--arm-start-source mc-command`从快照的`arm_command`建立`gripper` TCP及IK初值，
缺少命令流时拒绝；历史快照可显式指定`--arm-start-source state`进行实际反馈对照。
离线快照固定，首次Grip不能模拟现场重新采样命令；后续重按使用最后已接受理想目标。

`upper_body_online_core.DualArmOnlineIK`的`propose()`计算完整双侧候选，`validate_commit()`核对候选归属与版本，
`commit()`统一提交关节、SEW、分支与目标存储；调用方串行使用。无解、越限、分支改变或相邻／会话总变化超过10°时，
两侧状态均保持上一已接受值。现场每侧首次Grip同时建立该侧10°范围起点，后续重按保持此起点。
目标须位于`torso_link`，单位m和rad，未使能侧保持上一目标。
零增量保持种子；FK残差记录到日志。输入映射和IK均预检通过后才联合提交。
`OnlineTargetStore`保存完整目标、关联输入序号和单调接受时间，供后续持续发布读取。

输出沿用工作区`log/upper_body_*.jsonl`，记录真实快照、配置哈希、模型及TCP来源、XR输入、
映射目标、双侧候选、残差、耗时和接受／拒绝原因。首个失败停止回放，退出码1；成功接受至少一帧并完成读回后退出0。
当前入口不验证50Hz发布调度或实体跟随。容器内可单独运行状态契约检查：

```bash
python3 -m unittest discover -s src/functions/test -p 'test_upper_body_*.py'
```

### 现场只读IK与统计

`upper_body_teleop_observe`连接SDK、原始机器人反馈及MC的HAL手臂位置命令，保存XR与IK两份日志。
补偿关闭，拒绝`--execute`，不创建运动发布器。在机器人容器运行：

```bash
ros2 run aimdk_functions upper_body_teleop_observe \
  --device-id 实际设备标识 --input-config /实际路径/input.json \
  --mapping-config /实际路径/mapping.json \
  --xr-output /home/agi/aimdk-fsc/log/xr_observe.jsonl \
  --input-timeout 实测确定的秒数 --duration 60 \
  --load-description '实际工具和负载'
ros2 run aimdk_functions upper_body_teleop_stats /实际路径/upper_body_观测或回放.jsonl
```

`--input-timeout`显式配置接收、源时间停滞及排队年龄时限；跟踪失效、断链或时间回退终止本次观测。
缺少跟踪或重定位字段时按输入包配置及现场操作约定处理。反馈时限沿用0.5秒，可在0.2..1秒内配置。

机器人侧参考按以下三种情况建立，手柄增量始终相对于本次Grip按下时的手柄位姿：

| 情况 | 机器人侧TCP与IK参考 |
| --- | --- |
| 每侧首次Grip | 使用按下时新鲜的`/aima/hal/joint/arm/command`位置，建立该侧FK、IK种子、SEW和实验范围起点；分支从空状态开始跟踪 |
| 同一会话松开后重按 | 使用该侧最后已接受理想目标的FK，延续IK种子、SEW、分支和实验范围起点 |
| 中断后恢复 | 当前入口异常退出，操作者重启入口；重新采样HAL命令，各侧首次Grip重新建立参考 |

首次接管的参考与双侧IK候选共同提交，任一侧失败均不更新。未接管侧保持启动采样的HAL目标。
实际手臂反馈继续参与故障、源时间与时效检查；记录HAL命令与实际位置差及源时间差。
`mapping_reference`关联输入序号、各侧参考来源、参考关节及反馈序号；首次接管标记见`rebase_sides`。
夹爪初值取实际开度，接管期间按映射配置更新，松开后保持已接受开度。
当前只读入口补偿关闭；后续执行接入须沿用理想目标与补偿后发送目标分离的约定。

统计读回输入与结果关联、双侧残差及耗时P50/P95/P99/最大值；20ms计数仅用于观测计算负载。
统计退出0表示读回成功；`replay_complete`表示日志中每个输入均有成功结果及正常结束记录。
