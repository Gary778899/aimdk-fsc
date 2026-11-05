# 自定义功能工具

适用：X2 Ultra、官方 v1.3.0 模型、机器人 Ubuntu 22.04 / ROS 2 Humble / AimDK 环境。
ROS 2 包名为 `aimdk_functions`，使用 `ament_python`，依赖 NumPy >=1.21。

运动统一入口为 `upper_body_control joint/tcp/gripper`。保留 `volume_control`、
`arm_compensation_probe` 和 `upper_body_kinematics_check` 独立工具；
`upper_body_home` 提供home位置录制与回位。
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

## Home位置录制与回位

`upper_body_home record` 在操作者切换至URS／RUNNING、尚未开始遥操时，
采样MC的HAL双臂位置命令与左右夹爪实际开度，写入JSON配置。
录制为只读操作，重复录制覆盖指定配置；录制期间应停止其他上肢命令发布者。
录制配置与执行入口已在机器人端使用；跟踪误差观测策略修改仅完成本地静态、语法检查，待机器人复验。

同步并构建`aimdk_functions`、加载环境后，在机器人容器工作区根目录执行：

```bash
cd /home/agi/aimdk-fsc
# 手动从SD切换至URS，等待RUNNING，保持预期home姿态后录制。
ros2 run aimdk_functions upper_body_home record

# Ctrl-C结束upper_body_teleop_observe --execute，再查看回位起点、目标和夹爪开度。
ros2 run aimdk_functions upper_body_home run

# 确认回位空间后执行；示例用5秒，可增加时长降低运动速度。
ros2 run aimdk_functions upper_body_home run --duration 5 --execute
```

配置默认是**当前工作目录**下的`data/home.json`；可用`--config /绝对路径/home.json`
选择其他文件。机器人工作区`data/`受现有同步排除保护，使用“机器人：下载采集数据”任务下载备份。
避免将机器人录制配置写入受`rsync --delete`管理的源码目录。
日志沿用工作区`log/`，可用`--log-dir`指定。

JSON格式版本为`schema_version=1`，记录`robot_model=x2-ultra`、`urdf_version=v1.3.0`、
`arm_unit=rad`；`arm_positions`按完整14关节名称保存位置，读取时按名称映射；
`hand_openings.left/right`为0..1开度，0闭合、1全开。
`recording`保存模式、输入话题、各流源时间及单调接收时间、实际双臂反馈和日志路径；
`recorded_at_utc`保存录制时间。各流独立采样，不声明硬件同步。

回位从当前HAL双臂位置命令和夹爪实际开度起步，双臂和夹爪共用起终点速度、加速度为零的
五次插值，默认5秒，`--duration`接受有限正秒数。额外位置补偿固定关闭。
本命令使用绝对14关节目标，允许超过`joint`入口的单关节±10°增量实验范围；
双臂起终点和发送目标沿用现有关节执行限位，不裁剪目标。
发布与完成判定复用现有执行器：50Hz原生上肢命令，运动期间跟踪误差仅记录，
双臂终点要求处于±5°范围并持续确认0.3秒、额外等待最多2秒。
`motion_begin.tracking_policy=observation_only`标识home策略；每条`command_published`
记录实际关节位置、相对本次发送目标的`tracking_error_rad`、超5°观测标记及所用反馈序号与时间。
其他单次动作继续沿用±5°跟踪中止门槛。
夹爪只发送插值开度并记录终点反馈，不判定夹爪到位。
启动等待`--timeout`默认5秒、范围1..10秒；`--feedback-timeout`默认0.5秒、范围0.2..1秒。
异常和中断沿用既有停发与错误锁存，模式由操作者切换。
`xr_watch`仅接收输入，可保持运行；执行home前需结束持续发布保持目标的遥操执行进程，松开Grip仍会发布。

回位轨迹为关节空间插值，执行前检查双臂与身体、环境的中间路径，并确认夹爪改变开度时的物体状态。
首次机器人验证：空爪录制、核对JSON与同次日志中的采样值；只读预览应无命令发布；
在小幅偏离home后显式执行，核对首帧等于当前采样目标、最终发送值等于配置，
并记录实际双臂误差、夹爪开度及完整执行结果。更大范围的回位逐步扩展验证。
跟踪策略复验使用已有命令与反馈偏差超过5°的起点：首条home命令应与采样的HAL命令一致，
误差超5°时应记录并继续发布；到达轨迹终点后，仍须按上述终点条件完成或超时报错。

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
| `upper_body_control_core.py` | 分侧目标、统一轨迹、双臂合并；`prepare_motion_target()`统一生成补偿目标并校验所选关节范围及未选目标保持 |
| `upper_body_execution.py` / `upper_body_mode.py` | 单发布器、时间调度、终点确认／有界模式查询 |
| `upper_body_cartesian_core.py` | 完整目标契约；端点与直线规划共用`_prepare_translation()`准备起点、模型、FK、SEW和位移，再分别求解与生成报告 |
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

预览和执行均调用`prepare_motion_target()`；执行器在实际发送前调用`check_publish_gap()`检查发布间隔。发布周期、间隔上限、终点稳定时间和额外等待时限集中在`upper_body_execution.py`，数值沿用上文约定。反馈错误、时效及模式检查继续由既有反馈和模式模块负责。本次整合仅完成本地静态检查，待机器人Humble容器构建与执行验证；遥操代码保持原实现。

校验职责：`CapturedTarget`检查完整目标的长度、有限值和夹爪开度，`for_arm()`检查单臂候选及所选臂起点/目标范围；规划层检查10°变化与路径分支，`validate_target()`检查动作所选关节范围和未选关节保持。IK适配器已校验求解结果的有限值，端点规划直接使用该候选；零位移及直线首点复用已校验的采样目标。补偿计算后仍校验新目标，执行器检查新鲜反馈及跟踪误差，消息组装层使用已校验目标和执行器生成的uint32序号。

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
| 关闭补偿，以5秒时长左右分别预览TCP零位移、单轴及XYZ位移的endpoint/line | 零位移保留采样目标；两类报告字段完整，line为5秒251点；所选臂超限及超过10°变化仍拒绝，未选臂保持 |

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
`commit()`统一提交关节、SEW、分支与目标存储；调用方串行使用。无解、越限或分支改变时，
两侧状态均保持上一已接受值。相邻及累计关节变化只记录，超过早期10°实验范围标记`exceeds_10deg_observation`，不据此拒绝。
现场每侧首次Grip建立累计变化参考，后续重按保持此参考。
目标须位于`torso_link`，单位m和rad，未使能侧保持上一目标。
零增量保持种子；FK残差记录到日志。输入映射和IK均预检通过后才联合提交。
`OnlineTargetStore`保存完整目标、关联输入序号和单调接受时间，执行模式的发布侧读取同一快照。

输出沿用工作区`log/upper_body_*.jsonl`，记录真实快照、配置哈希、模型及TCP来源、XR输入、
映射目标、双侧候选、残差、耗时和接受／拒绝原因。首个失败停止回放，退出码1；成功接受至少一帧并完成读回后退出0。
当前入口不验证50Hz发布调度或实体跟随。容器内可单独运行状态契约检查：

```bash
python3 -m unittest discover -s src/functions/test -p 'test_upper_body_*.py'
```

### 现场只读IK与统计

`upper_body_teleop_observe`连接SDK、原始机器人反馈及MC的HAL手臂位置命令，保存XR与IK两份日志。
`--xr-output`指定的文件已存在时，自动依次使用`文件名_1.jsonl`、`文件名_2.jsonl`，保留已有日志；终端显示实际XR路径。
当前现场映射配置使用Grip时固定的头显水平朝向，须同时开启Head及Controller tracking；
操作契约见[输入包映射配置](../remote_control/README.md#映射配置)。
历史无Head日志回放须使用`reference_mode=world`的配置。
朝向验收先只读：面向任意方向，按Grip后依次向自身前、左、上移动，目标增量应分别以正X、正Y、正Z为主；
保持手柄位姿只转头，目标应保持；重新Grip后按新的头显朝向对齐。左右侧分别验证。
`mapping_candidate`记录每侧`head_yaw_rad`及实际`mapping_basis`，对应参考只在候选接受时生效。
默认只读、补偿关闭；显式`--execute`开启下节定义的持续发布。在机器人容器只读运行：

```bash
ros2 run aimdk_functions upper_body_teleop_observe \
  --device-id 实际设备标识 --input-config /实际路径/input.json \
  --mapping-config /实际路径/mapping.json \
  --xr-output /home/agi/aimdk-fsc/log/xr_observe.jsonl \
  --duration 60 \
  --load-description '实际工具和负载'
ros2 run aimdk_functions upper_body_teleop_stats /实际路径/upper_body_观测或回放.jsonl
```

首帧及停流期间持续等待新XR帧，记录接收间隔、帧年龄和积压；等待期间不重复求解缓存帧。
现场输入由独立接收线程持续解析、校验并记录全部回调和完整帧。
普通位姿以20Hz为上限选取最新完整双手柄帧，未选中的普通帧合并，不补算错过的周期。
两侧Grip按下／松开的完整帧按接收顺序保留，优先于普通位姿处理，不受50ms选帧间隔限制。
边沿帧只建立参考或保持双侧已接受位姿与夹爪，不推进运动求解；普通帧才更新运动目标。
普通候选求解期间出现任一侧新Grip边沿时，丢弃整个双侧候选，继续处理边沿；已接受状态保持。
合并前逐帧检查跟踪、连接、设备／参考空间及源时间回退，异常终止会话。
原始XR日志保留完整输入；IK日志的`xr_input`只包含被选中的帧，离线回放仍按完整输入FIFO处理。
`--duration`包含等待时间，省略时运行至Ctrl-C。首帧等待期间可在头显选择Reconnect并开启Controller／Send。
`--timeout`用于初始机器人反馈等待，默认5秒、范围1..10秒；执行时还复用到模式查询及DDS匹配。
`--input-timeout`为可选观测实验参数，显式指定时才按接收间隔、源时间停滞及帧年龄中止。
跟踪失效、断链或时间回退终止本次观测。
缺少跟踪或重定位字段时按输入包配置及现场操作约定处理。反馈时限沿用0.5秒，可在0.2..1秒内配置。

机器人侧参考按以下三种情况建立，手柄增量始终相对于本次Grip按下时的手柄位姿：

| 情况 | 机器人侧TCP与IK参考 |
| --- | --- |
| 每侧首次Grip | 使用按下时新鲜的`/aima/hal/joint/arm/command`位置，建立该侧FK、IK种子、SEW和累计变化参考；分支从空状态开始跟踪 |
| 同一会话松开后重按 | 使用该侧最后已接受理想目标的FK，延续IK种子、SEW、分支和累计变化参考 |
| 中断后恢复 | 当前入口异常退出，操作者重启入口；重新采样HAL命令，各侧首次Grip重新建立参考 |

首次接管的参考与双侧IK候选共同提交，任一侧失败均不更新。
执行模式首次接管时刷新完整14轴HAL参考，尚未接管侧保持此目标；只读模式未接管侧保持启动采样值。
实际手臂反馈继续参与故障、源时间与时效检查；记录HAL命令与实际位置差及源时间差。
`mapping_reference`关联输入序号、各侧参考来源、参考关节及反馈序号；首次接管标记见`rebase_sides`。
独立接收后，HAL参考在主线程处理Grip按下帧时采样；保留该帧原始接收时间及参考反馈时间供核对。
夹爪初值取实际开度，接管期间按映射配置更新，松开后保持已接受开度。
只读观测保留理想IK目标；执行模式默认开启补偿，分别记录理想目标和实际发送目标。

统计读回输入与结果关联、双侧残差及耗时P50/P95/P99/最大值；20ms计数仅用于观测计算负载。
统计退出0表示读回成功；`replay_complete`表示每个选中输入均有成功或被新Grip边沿取代的结果及正常结束记录。
`input_selected`记录边沿标记、选帧年龄与累计合并数；`ik_superseded`关联丢弃候选的输入序号。
统计中的`input_selection`汇总选帧和合并计数，`superseded_inputs`列出被边沿取代的普通候选。

### 遥操持续执行

`upper_body_teleop_observe --execute`复用原生`/mc/upper_body_command`，不切换模式。
位置偏置补偿默认开启，`--no-compensation`用于关闭对照。复用[既有补偿算法](#位置偏置补偿与负载)，
刚度40 N·m/rad、偏置限幅±12°；本轮附加载荷固定0kg，对应gripper空载。
开启时订阅骨盆IMU及具名腰反馈；只读模式不启用补偿计算或新增这两路订阅。
每侧首次Grip后按`--compensation-duration`秒的五次曲线从零引入，默认5秒，独立于会话`--duration`。
尚未接管侧保持目标，已接管侧松开及重按Grip均保持补偿进度；异常重启后重新从零引入。
理想关节和已接管侧随同一缓存快照提交，50Hz发布侧计算补偿，补偿结果不写回IK种子或Grip参考。
启动时通过`GetMcAction`确认URS/RUNNING，沿用实际MC模式、故障及反馈源时间检查。
首次Grip前完成DDS匹配但不发布；首次Grip时重新采样完整HAL手臂位置命令，
建立双侧种子后立即发送与该命令一致的零增量首帧。
之后以50Hz发送最新完整已接受目标；输入接收、位姿选择和Grip处理见上节。
松开Grip保持最后目标并继续发布，重按延续同一发布会话。

执行模式的`--input-timeout`默认0.5秒，可显式调整；首次Grip前不应用输入时限。
接管后，接收线程按原始时间检查回调间隔、接收处理年龄及已知源时间停滞。
发布线程同时检查接收线程异常和主线程已选帧的原始接收年龄；后者包括选帧／求解等待。
提交前复核选中帧年龄，任一检查超过同一`input-timeout`即停发并退出。
接收线程的新帧不会刷新主线程处理时间。`command_published.input_age_s`记录主线程选中帧年龄，
`latest_receive_age_s`记录接收线程最近完整帧年龄，两者均使用原始接收时间。
0.5秒是首轮试验值，依据此前接收最大间隔约75ms选取，实际响应待真机验证。
显式断链、跟踪失效、时间回退、IK失败、反馈故障／失效、Ctrl-C或会话结束均停发。
退出后由操作者重启，从新鲜HAL命令重新建立参考；停发后的实体保持与超时行为须现场记录。

ROS反馈及发布定时器运行在独立单线程执行器，接收线程独占XR录制器，主线程处理映射并求解IK。
发布侧也检查输入时限，IK阻塞时可独立停发；接收边沿、目标联合提交与发布／停发共用锁。
退出时先停发、结束接收线程，再关闭SDK、排空原始回调并关闭日志。
反馈采样和执行JSONL写盘分别加锁。发布周期20ms、发布间隔上限80ms、
启动漂移±3°复用既有原生执行器约定；遥操关节跟踪误差仅作观测，超过±5°继续发布。
关节限位、反馈故障／时效、输入时限和发布间隔检查继续生效。
线程隔离后的实际周期、GIL及同步日志开销由容器日志确认。

`teleop_execution_begin.tracking_policy=observation_only`标明本轮跟踪策略，
`tracking_observation_limit_rad`为既有5°的rad值，仅用于观测标记。
`command_published`记录本次使用的`actual_arm_rad`，`tracking_error_rad`为实际位置减本条发送目标，
`previous_target_tracking_error_rad`为同一反馈减此前最近成功发送目标；后者首帧为null。
前目标及序号分别保存在`previous_sent_arm_rad`和`previous_command_sequence`，首帧为null。
这两项误差按发布回调取得的反馈计算，不代表硬件同步或目标到位判定。
`tracking_exceeds_5deg_observation`及`previous_target_tracking_exceeds_5deg_observation`分别标记
是否有任一关节超过5°，不改变发布状态。验收时超过5°应持续记录并发送，反馈故障或输入失效仍停发。
`upper_body_teleop_stats`分别输出两种最大绝对关节误差的分布、超5°帧数和观测样本数；
历史日志缺失前目标字段时不补样本。该变更仅适用于遥操，`upper_body_control`沿用既有跟踪及终点门槛。

本次20Hz策略待机器人容器验证：先只读运行，分别持续移动单侧及双侧手柄，确认
`selection_age_s`和`input_age_s`不持续增长；快速松开／重按应依次记录`released`和`reference_established`。
执行时确认普通候选跨Grip边沿记录`ik_superseded`且目标保持，发布间隔仍符合既有80ms上限。

在已手动进入URS/RUNNING、Service运行且PICO持续发送时，在机器人容器执行：

```bash
cd /home/agi/aimdk-fsc
bash scripts/build_overlay.sh remote_control aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash

TELEOP_LOG_DIR="/home/agi/aimdk-fsc/log/teleop_$(date +%Y%m%d_%H%M%S)"
ros2 run aimdk_functions upper_body_teleop_observe \
  --device-id TestDevice \
  --input-config /home/agi/aimdk-fsc/src/remote_control/config/input.json \
  --mapping-config /home/agi/aimdk-fsc/src/remote_control/config/mapping.json \
  --sdk-library /opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so \
  --xr-output "$TELEOP_LOG_DIR/xr_input.jsonl" \
  --log-dir "$TELEOP_LOG_DIR" \
  --timeout 10 --input-timeout 0.5 --duration 60 --display-interval 1 \
  --load-description 'gripper，空载' --execute

TELEOP_IK_LOGS=("$TELEOP_LOG_DIR"/upper_body_*.jsonl)
ros2 run aimdk_functions upper_body_teleop_stats "${TELEOP_IK_LOGS[0]}"
```

首轮依次验证零增量保持、右臂小幅平移及旋转、松开／重按、左臂及双臂动作。
预期首帧14轴命令匹配接管HAL参考，补偿从零引入；松开保持理想目标，补偿继续根据姿态反馈计算，重按连续。
关闭Send后预期约0.5秒加调度开销内停发；记录实体响应，重新运行后可从当前HAL目标接管。
`command_published`记录输入序号、接受时刻、发送位置、发布间隔、输入年龄及跟踪误差。
`compensation.arms`记录各侧引入进度、重力估计、原始／限幅偏置及IMU／腰源时间。
首次零增量发送的输入序号为空；后续发送关联最近的已接受IK输入。
统计增加`execution`及发布耗时分布；`accepted_inputs_not_published`列出50Hz采样未发送的IK输入，
`unknown_published_inputs`正常应为空。当前只完成本地静态检查，容器执行及实体跟随待验证。

### 跟随误差实验日志与离线分析

现场入口增加`--record-body`，独立于补偿开关记录骨盆IMU及腰原始消息。
`--no-compensation --record-body`下两路消息带`observation_only=true`，保留原始字段、接收时间及逐流serial；
它们不参与采样就绪、反馈时限和错误中止。未收到消息时没有对应记录，分析报告将计数列为0。
开启补偿时仍使用既有必需IMU／腰反馈及校验，记录选项不改变其约束。

遥操`command_published`新增：

| 字段 | 定义 |
| --- | --- |
| `arm_feedback_serial` | 本次实际使用的arm反馈序号，对应同日志`feedback.stream=arm`的serial |
| `arm_feedback_stamp_s` | 该反馈的原始源时间，秒 |
| `arm_feedback_received_monotonic_s` | 该反馈的本机单调接收时间，秒 |
| `publish_begin_monotonic_s`／`publish_end_monotonic_s` | `publisher.publish()`调用前后的本机单调时间；不是机器人内部执行时刻 |

`upper_body_teleop_analyze`只读日志，不导入ROS或调用SDK。核对日志URDF摘要并加载日志中的完整工具变换，
输出分侧CSV、`summary.json`与解释报告，可选输出PNG。运行示例：

```bash
# 机器人容器构建安装后运行；也可使用下方本地入口。
ros2 run aimdk_functions upper_body_teleop_analyze /实际路径/执行日志.jsonl \
  --side both --start 5 --end 25 --output /新的分析目录

# 本地项目根目录：只需NumPy；--plots另需matplotlib。
PYTHONPATH=overlay/src/functions python3 -m aimdk_functions.upper_body_teleop_analyze \
  /实际路径/执行日志.jsonl --side right --output exports/跟随误差_会话1 --plots
```

窗口相对首条发布调用开始时间，历史日志缺该字段时用首条发布日志记录时间。
输出含每轴发送目标误差、理想目标误差和实际偏置，单位度；原始关节rad；
名义TCP的IK、补偿几何变化、反馈跟踪三项位置分解与任务误差，单位mm。
统计按发布计数，同时报告唯一arm反馈数、复用数及缺少反馈标识的条数。
`startup`为首次零增量发送，`ramp`为该侧补偿渐入，`holding`为已接受目标中该侧Grip释放，
其余为`tracking`；`holding`仅定义目标保持，不证明实体稳定。分析静态末5秒时显式指定窗口。

新日志可用`--delay-max 秒数`开启命令历史的非负表观延迟与常值偏差拟合。
搜索上界从停走响应确定，步长默认2ms；前2/3独立反馈拟合，后1/3验证。
按反馈serial去重并使用单调接收时间，命令采用零阶保持；只使用全部候选延迟共同有效的样本，
命令间隔上限默认80ms。报告零延迟与对齐后的误差、验证残差和搜索边界标记。
示例中确定上界后，在上述命令添加`--delay-max 实测上界秒数`；该估计包含传输、采样与机器人响应。
历史日志仍可计算误差分解；缺少明确发布时刻或反馈关联时，延迟结果为`unavailable`。
输出目录必须新建，避免覆盖已有实验结果。全部指标按名义模型解释，实体精度及世界系误差需要独立测量。
完整操作与判别见[误差实验方案](../../../docs/teleop_tracking_error_experiments.md)。

### 映射检查与原节奏回放

回放和现场只读入口均支持`--display-interval 秒数`，默认1，0关闭终端映射输出。
Grip建立参考、松开和失败立即显示。输出标明输入序号及接受／拒绝结果，左右分别显示
Grip、Trigger、相对本次Grip参考的手柄／TCP平移m和旋转向量度。
旋转向量在各自父坐标系表达，180°时旋转轴整体反号等价；完整旋转矩阵保存在日志中。

`mapping_candidate`逐帧保存每侧事件、手柄与机器人参考位姿、输入增量、TCP增量、
TCP目标及夹爪开度。它描述待提交候选，接受情况以同序号`ik_accepted`／`ik_rejected`为准。
`reference_established`表示本次候选建立参考，`tracking`表示沿用参考，
`released`表示松开，`holding`表示保持，`blocked`表示参考重置后等待松开Grip。

首个失败仍终止处理。`ik_rejected`包含失败输入、上一成功输入及目标、映射候选、
双侧IK状态与映射参考的前后快照；IK报告保留已算出的候选关节、步长、SEW及分支。
快照中的`last_consumed_seq`随输入消费推进，已提交参考与理想目标按联合提交规则保持。
统计的`first_failure`提供失败序号、上一成功序号及IK／映射参考是否保持。

回放默认`--replay-timing immediate`，尽快逐帧计算；`recorded`以首帧为锚点，
按原始接收单调时间差调度。处理落后时按FIFO消费，不跳帧、不重新设置时间锚点。
回放预载全部完整帧后开始调度；文件损坏、失联记录或时间异常在`replay_input_error`中报告，
不进入IK。输入格式错误包含文件、行号和可用输入序号。

```bash
ros2 run aimdk_functions upper_body_kinematics_check \
  --samples 1 --include-arm-command --load-description '实际工具和负载'
ros2 run aimdk_functions upper_body_teleop_replay \
  --xr-log /实际路径/xr_input.jsonl \
  --mapping-config /实际路径/mapping.json \
  --snapshot-log /实际路径/含HAL命令的快照.jsonl \
  --load-description '实际工具和负载' \
  --replay-timing recorded --display-interval 1
ros2 run aimdk_functions upper_body_teleop_stats /实际路径/回放日志.jsonl
```

`input_timing`和`frame_result_timing`记录调度、结果时刻及耗时，统计按时间语义分组：

| 分组／字段 | 定义 |
| --- | --- |
| `live` | 本进程SDK回调接收至消费／结果的时间；`backlog_events`为消费时剩余SDK事件数 |
| `replay_recorded` | 计划到达至消费／结果的时间；`backlog_frames`为当前帧之外已经到期的等待帧数 |
| `replay_immediate` | 只统计处理耗时，输入年龄、输入至结果耗时及等待帧数为空 |
| `receive_gap_s` | 原始相邻完整帧接收时间差，首帧为空 |
| `processing_age_s` | 实时为回调接收至归一化及原始帧写盘后的时间；回放为计划到达至本帧交付的时间 |
| `input_to_result_s` | 回调接收或计划到达至本帧结果记录完成的时间，不含后续终端显示 |
| `service_seconds` | 回放调度器交付本帧至结果记录完成的时间，包含同步诊断写盘 |

原始源时间与接收时间保持不变，回放计划时间单独记录；不计算跨设备延迟。
统计报告各组耗时、接收／结果间隔和积压的P50/P95/P99/最大值、样本数、积压首末值及增长次数，
并报告映射候选事件序号与各轴增量范围。`mapping_and_ik_seconds`包含输入及映射诊断写盘，
`dual_ik_seconds`单独计量双侧求解与校验。20ms计数只作负载观测。

容器验收按以下操作与预期结果执行：

1. 首次按Grip并保持手柄不动：手柄和TCP增量为零，现场该侧关节目标等于本次HAL命令参考。
2. 分别沿手柄三个轴平移、绕三个轴旋转：TCP方向符合`basis`，位移符合`translation_scale`；
   `relative`姿态遵循坐标变换，`fixed`姿态的TCP旋转增量为零。
3. 松开、移动手柄、重新按Grip：松开期间保持目标，重按后增量归零，参考延续最后接受的理想目标。
4. 回放含单侧无解或越限的输入：首个失败保存输入及候选，两侧IK状态保持，后续输入不再求解。
5. 对同一日志分别使用`immediate`及`recorded`：比较输入至结果耗时与积压，记录Orin负载。
   周期测量时可设`--display-interval 0`，诊断JSONL仍逐帧写盘。该测量覆盖只读链路，50Hz发布另行验证。

新增回归用例覆盖坐标反射、固定姿态诊断、失败状态读回及人工时钟下的FIFO积压；按上节容器测试命令执行。
本次另补充连续接收、20Hz最新位姿、双侧Grip边沿、跨边沿候选撤销及接收持续时IK阻塞的用例，待容器运行。
