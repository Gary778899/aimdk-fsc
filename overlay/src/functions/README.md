# 自定义功能工具

适用：X2 Ultra、官方 v1.3.0 模型、机器人 Ubuntu 22.04 / ROS 2 Humble / AimDK 环境。
ROS 2 包名为 `aimdk_functions`，使用 `ament_python`，依赖 NumPy >=1.21。

运动统一入口为 `upper_body_control joint/tcp/gripper`。保留 `volume_control`、
`arm_compensation_probe` 和 `upper_body_kinematics_check` 独立工具。
本轮统一入口与双臂调度只完成本地静态、语法检查，尚未执行机器人容器构建或实机验证。
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
同步排除会保护旧安装产物；新增入口必须重新构建，旧可执行文件可能仍出现在
`ros2 pkg executables aimdk_functions`。确认实际安装布局和 overlay 来源后清理旧产物，
不要继续运行旧入口。保持现有同步排除规则。

## 控制约定

所有动作默认只读，仅显式 `--execute` 创建发布器。执行前由操作者进入
`UPPERBODY_REMOTE_SPLIT`（URS）并等待 `RUNNING`，确认动作空间及无其他节点同时发布上肢目标。
启动时有界查询 GetMcAction，执行期间持续检查原生模式反馈。
只读规划允许其他模式；规划后刷新反馈，再检查起点漂移与时效后开始执行。

每次发送 `/mc/upper_body_command` 完整左7右7关节 rad 目标和模式1的左右夹爪开度。
反馈按名称映射。未选关节与夹爪保持本次采样值；开启补偿时所选臂的七个关节均可能改变。
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
反馈QoS沿用best_effort/transient_local，补偿传感器best_effort/volatile，发布深度10。

发布50Hz，间隔上限80ms；全部关节跟踪与终点容差±3°，终点新反馈连续确认0.3秒，
轨迹或夹爪持续发送结束后额外最多等待2秒。终点须已实际发布完整目标，补偿进度达到1。
到位比较对象为实际发送目标，原理想轨迹和反馈FK另行记录。
小于3°的动作可能在实际运动很小时满足确认，不能据此认定精确执行了输入增量。
`--timeout`还用于启动反馈、服务发现/响应与DDS匹配的各自有界等待。

日志默认在自动探测的工作区 `log/upper_body_*.jsonl`，失败时用 `--log-dir`明确指定。
按事件增量写入并flush，包含完整原始反馈与时间、代码/模型哈希、限位、请求、分侧IK/SEW/分支、
路径残差、补偿参数/偏置/源时间、理想与实际发送目标、周期耗时、反馈FK及退出原因。
未知SDK/固件版本标记unknown，真实版本由操作者核对。日志复用现有“机器人：下载日志”任务。

## 实现结构与独立观测工具

| 模块 | 职责 |
| --- | --- |
| `upper_body_control.py` | 参数、单次生命周期与日志 |
| `upper_body_control_core.py` | 分侧目标、统一轨迹、双臂合并与补偿组装 |
| `upper_body_execution.py` / `upper_body_mode.py` | 单发布器、时间调度、终点确认／有界模式查询 |
| `upper_body_move_core.py` / `upper_body_line_core.py` | 端点／直线纯IK规划 |
| `upper_body_ros.py` / `dual_arm_control_core.py` | ROS反馈适配／纯解码、时效、错误锁存、关节插值 |
| `upper_body_compensation.py` | 分侧位置偏置计算 |
| `upper_body_diagnostics.py` | 共用增量日志 |

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
