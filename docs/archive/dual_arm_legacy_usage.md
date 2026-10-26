# 迁移前手臂实验用法与验证事实

本页保存功能迁移前的完整 README，含用户已确认的验证结果。以下旧双臂、肩方向和夹爪命令已移除，不作为当前运行方法；当前入口见 [功能包 README](../../src/functions/README.md)。历史验收结论不自动推广到新增七关节单次控制。

# 自定义测试工具

ROS 2 包名为 `aimdk_test`，使用 `ament_python`。可执行脚本和辅助模块位于
`aimdk_test/`，包配置位于当前目录。`head_test` 和 `arm_test` 默认只读；
原有的运动逻辑和显式 `--execute` 开关保持不变。

由用户同步工作区后，在机器人开发板的 Humble 容器中，从工作区根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to aimdk_test aimdk_functions
source install/local_setup.bash
ros2 run aimdk_test head_test --help
ros2 run aimdk_test arm_test --help
ros2 run aimdk_test dual_arm_control --help
ros2 run aimdk_test shoulder_direction_test --help
ros2 run aimdk_test gripper_close --help
```

如果使用自定义构建、安装目录，请沿用对应的 colcon 参数，并 source 对应的
`local_setup.bash`。默认日志写入检测到的工作区 `log/`；工作区无法检测时，
使用 `--log-dir` 指定路径。

## 双臂肩关节与夹爪顺序控制

`dual_arm_control` 使用 `/mc/upper_body_command`，保留下半身原生运控。
发布端参照 SDK `src/py_examples/py_examples/upper_body_control.py`：
使用 `create_publisher(..., 10)` 的默认 QoS、20 ms 定时器（目标 50 Hz），
每次发送填写当前 ROS 时间戳，并递增命令序号。阶段循环更新完整目标，
定时器发送最新目标；每次发送前仍检查反馈和 URS 模式。定时器与订阅由同一
执行器处理，不能保证在进程调度或通信阻塞时维持严格 50 Hz，超过 80 ms
的发布间隔仍中止。SDK 接口章节表列 `BEST_EFFORT + VOLATILE`，而 Python
上肢示例使用默认 QoS；本脚本的发布端按该示例对齐，实际连接仍需实机验证。

该官方上肢示例没有创建反馈订阅，也没有反馈年龄或到位判定逻辑。
本脚本的反馈订阅继续使用原手臂测试的 `BEST_EFFORT + TRANSIENT_LOCAL`；
SDK `motocontrol.py` 展示的是 `BEST_EFFORT + VOLATILE` 的 HAL 订阅及控制，
不能把其中的 HAL 发布路径搬到本脚本。双臂起点、到位误差和时间有效性
仍由本脚本单独检查。只读阶段尚未创建命令发布器，接收超时不能归因于其发布配置。

首先调用 `/aimdk_5Fmsgs/srv/GetMcAction`（`get_mc_action` 示例使用的接口），
仅接受 `UPPERBODY_REMOTE_SPLIT`（URS）且状态为 `RUNNING=100`。
其他模式会提示先手动切换并退出；脚本不调用 SetMcAction。
切换动作由操作者执行，之后重新启动本脚本。只读检查方式：

```bash
ros2 run aimdk_test dual_arm_control
```

启动查询参照官方 `get_mc_action` 示例：单次请求等待最多 0.25 秒，未响应则
重新发送只读查询；服务发现和查询响应分别受 `--timeout`（默认 5 秒）约束。
仅在启动时查询，首次有效响应确认 URS/RUNNING 后不再发送服务请求。
日志记录发现耗时、每次请求、重试及响应耗时，
明确区分服务发现超时、响应超时与 ROS 停止。错误响应或非 URS/RUNNING
直接退出。启动后的观察和执行依赖 `/aima/mc/common/state` 的新鲜反馈，
持续要求 body=STAND、模式=URS、状态=RUNNING；话题中出现不符状态时锁存错误并中止。
不再进行后台 GetMcAction 轮询，也不再使用服务响应的 0.3 秒超时或 0.4 秒确认有效期。

当前将“向上”具体实现为双肩 **pitch 的前向抬臂**，对应
`left_shoulder_pitch_joint` / `right_shoulder_pitch_joint`，命令槽位为 0 / 7。
不同时改变 shoulder roll 或 yaw。2026-10-24 用户实机确认当前机器人左右肩
pitch 正增量均向下，因此上抬使用负增量。双臂脚本已默认设置
`--left-up-sign -1 --right-up-sign -1`，两肩目标均为各自起点 **减 5°**。
实测记录、到位误差及未完成的验证见工作区 [test.md](../../test.md)。
若需要侧向抬臂，应先明确 shoulder roll 的实机方向与适用限位，不能直接套用本流程。

如需重新验证方向，先在现场确认夹爪空载及动作空间，分别运行：

```bash
# 默认只读：查看全部关节起点，不发送命令。
ros2 run aimdk_test shoulder_direction_test --side left

# 左右分别执行 pitch +1°（当前机器人已实测为向下）：正常流程为 5 秒去程、保持 3 秒、5 秒返回。
ros2 run aimdk_test shoulder_direction_test --side left --delta-deg 1 --execute
ros2 run aimdk_test shoulder_direction_test --side right --delta-deg 1 --execute

# 负方向为当前机器人上抬方向；这里给出左肩 -5° 上抬验证。
ros2 run aimdk_test shoulder_direction_test --side left --delta-deg -5 --execute
```

每次验证也要求预先进入 URS，并等待 3 秒记录起点。验证脚本仅移动指定侧肩
pitch，完整重发其余关节实测起点和两侧夹爪当前开度，不主动开闭夹爪。
去程只对实际移动的肩 pitch 检查到位，容差为偏移绝对值的一半且最多 3°，
使小幅验证时未发生运动不会被误判为去程到位；其余保持关节及回程容差为 ±3°。
`--delta-deg` 可设为带符号的
0.5..5°，默认 1°。观察正偏移的实际方向：若为所需抬升，则该侧 sign 填 `1`；
若为下降，则填 `-1`，并使用 `--delta-deg -1` 验证该侧抬升方向后再执行正式流程。
仅靠关节角反馈不能自动判断末端是否在物理空间向上，需操作者现场确认。

正式流程采用已确认的左右负方向，不再要求每次填写方向参数：

```bash
ros2 run aimdk_test dual_arm_control --execute
```

方向参数仍可显式覆盖，以便其他机体重新标定；本次方向结论不自动推广到其他机型和姿态。
本次右肩正向验证已观察到运动，但因去程误差 1.605° 超过 1°门槛而中止，
没有执行回程；不能将方向确认理解为完整往返、双臂上抬或夹爪顺序流程验收通过。
该 1°门槛现已调整：双臂各阶段为 ±3°，5°方向验证的去程为 ±2.5°。
重力补偿不足是用户提出的可能原因，尚未确认固件内部补偿状态及误差成因。

正式流程为：

1. 查询确认 URS / RUNNING，等齐有效且源时间推进的双臂、夹爪、MC 反馈，
   再观察 3 秒，记录按关节名重排的全部 14 个关节起点；记录时要求夹爪静止。
   观察期间检查反馈有效性，不附加全部关节在小范围内静止的精度门槛。
2. 创建发布器后有界等待 DDS 订阅匹配（最多 `--timeout` 秒），
   然后直接从实测起点开始五次插值，同时将两侧肩 pitch
   沿各自确认方向移动 5°。单程默认 5 秒，`--leg-seconds` 可设为 5..10 秒。
   其余 12 个关节保持起点，夹爪保持采样开度。
   去程终点要求全部关节误差不超过 3°并连续稳定 0.3 秒。
   不再有抬肩前的 `initial_hold` 精密到位步骤。
3. 持续发布双臂抬升位置，同时发送模式 1 的 `hand_pos=[1, 1]`，
   两爪全开到位后再保持 3 秒；然后发送 `[0, 0]` 闭合两爪。
   两侧开度误差均需不超过 0.02、状态为静止且连续稳定 0.3 秒。
   堵转、物体掉落或故障会中止，因此此流程面向空爪，不用于夹持物体。
4. 闭合到位后平滑返回全部双臂起点，全部 14 个关节误差不超过 ±3°
   并连续稳定 0.3 秒才报告完成。夹爪保持关闭。

命令约 50 Hz，肩 pitch 路径采用 SDK `motocontrol.py` 中的
`[-3.08, 2.04] rad` 向内保留 1°的范围；该范围未经当前真机验收，
也不是碰撞检查。其余关节只保持实测起点，不沿用旧腕部实验的姿态窗口。
首次接收命令后、各阶段终点均要求新的源时间反馈。运动中全部关节跟踪误差
超过 3°、发布间隔超过 80 ms、反馈过期/冻结/回退、硬件故障、站立状态丢失
或 MC 状态话题中的 URS/RUNNING 状态变化均中止。
手爪到位超时默认 5 秒，可用 `--timeout` 设置
为 1..10 秒；手臂终点额外等待最多 2 秒。

双臂控制和肩方向验证分别用 `--mc-feedback-timeout`、`--arm-feedback-timeout`
和 `--hand-feedback-timeout` 配置 MC、手臂和夹爪的反馈时限。三者默认均为
**0.5 秒**，范围均为 0.2..1 秒，**在只读、起点观察和运动执行阶段持续生效**。
每项时限同时约束该流的接收间隔、源时间未推进时长和源时间年龄；仅改接收间隔
会被后两项继续拦截。执行前不会再将手臂和夹爪强制改回 0.2 秒。
可以先只读检查：

```bash
ros2 run aimdk_test shoulder_direction_test --side right --delta-deg 5 \
  --mc-feedback-timeout 0.5
```

MC 状态标称 10 Hz；实机 `ros2 topic hz` 曾观察到手臂约 500 Hz、最大接收间隔 0.333 秒，
脚本起点观察阶段还曾捕获夹爪约 0.210 秒未更新。
这只能证明该订阅端出现过长间隔，不能区分发布端停顿、传输延迟或回调调度，
也不能由累计最大值判断重复发生次数。0.5 秒是据此选取的调试软件阈值；
2026-10-24 完整双臂与夹爪流程已使用该配置验证通过，记录见 [test.md](../../test.md)。
该流程结果不等于专项断流／冻结测试通过，放宽时限会延后相关检测。
显式指定三个时限并执行右肩方向验证：

```bash
ros2 run aimdk_test shoulder_direction_test --side right --delta-deg 5 \
  --mc-feedback-timeout 0.5 --arm-feedback-timeout 0.5 \
  --hand-feedback-timeout 0.5 --execute
```

ROS 时钟停滞上限仍为 0.2 秒，源时间回退、未来时间戳及硬件故障检查继续保留。
MC 模式状态的时效由 `--mc-feedback-timeout` 约束，GetMcAction 仅承担启动检查。
上述三个参数独立于 80 ms 命令发布间隔及关节位置误差判定。
执行阶段保留原有样本、源时间推进记录及错误锁存，
不重新初始化监视器，也不改写反馈时间戳。日志记录启动参数和执行时实际生效的阈值。
旧 `head_test` / `arm_test` 的默认时限保持为 0.2 秒。

`shoulder_direction_test` 直接调用 `dual_arm_control.main(..., probe=True)`，
两者共享反馈订阅、模式检查、发布器和阶段执行代码，没有另一份需手动同步的保护实现。
当前共用配置如下：

| 检查 | 双臂控制 | 单肩方向验证 |
| --- | --- | --- |
| MC / 手臂 / 夹爪反馈时限 | 各默认 0.5 秒，全程生效 | 相同 |
| 抬肩前全部关节 ±0.3°稳定等待 | 已删除 | 已删除 |
| 后台 GetMcAction 轮询和 0.4 秒确认有效期 | 已删除，仅启动查询 | 相同 |
| 运动跟踪、保持及回程位置容差 | ±3° | ±3° |
| 去程到位容差 | ±3° | 偏移的一半，最多 3°；5°测试为 ±2.5° |
| 到位连续确认时间 | 0.3 秒，无额外启动精度门槛 | 相同 |
| 命令发布间隔保护 | 80 ms，独立于反馈接收时限 | 相同 |
| 夹爪开度容差、静止判断及硬件故障检查 | 继续保留 | 相同 |

位置容差集中定义在 `dual_arm_control_core.ARM_POSITION_TOLERANCE`，
避免去程、保持和回程各自残留更严格阈值。放宽的是误差允许范围，目标仍为上抬 5°，
并不代表实现了重力补偿或已完成新容差的实机验收。

正常结束或异常退出时停止本脚本发布，模式仍由操作者管理；异常时不自动返程。
停止发布后的保持和超时行为尚未实机验证，不能把停发当作急停或保证继续保持姿态。
执行前需现场确认没有其他节点同时向上肢话题发送目标。
日志前缀分别为 `dual_arm_` / `shoulder_direction_`，记录完整起点、原始反馈、
命令、模式查询及阶段到位结果。2026-10-24 用户已在机器人开发板容器完成
双臂上抬、两爪全开、保持、闭合及回程流程验证；具体范围见 [test.md](../../test.md)。
后续代码或参数变更需分别记录构建与验证结果。

旧版本曾在抬肩前要求全部 14 个关节达到 ±0.3°，这不是官方上肢示例的步骤，
现已移除。当前打印“肩 pitch 偏移”后直接进入 `raise` 阶段。
每个阶段会打印名称、轨迹时间和终点容差。超时信息会列出超差关节及角度、
两侧夹爪开度/状态/速度、新源反馈条件、实际发布帧数和 DDS 订阅匹配数；
JSONL 同时记录 `segment_begin` / `endpoint_timeout`。
DDS 匹配及本地发布计数只能证明存在接收端和本地发布调用，不能证明固件接受或执行了命令。

## 独立夹爪开度控制

`gripper_close` 是独立命令，复用肩方向测试的反馈与发布执行器，命令名保留兼容。
只在启动时通过 GetMcAction 确认 URS/RUNNING；未处于该模式则提示手动切换并退出，
启动后使用 MC 状态话题检查模式，不后台轮询服务。
等待 3 秒记录完整双臂位置及左右夹爪开度后，`--execute` 才创建发布器。
双臂所有关节持续保持记录的起点，对选定夹爪发送模式 1 的目标开度。
`--opening` 接受 0..1 的有限数值：`0` 闭合，`1` 完全张开，中间值为部分张开；默认 `0`。
默认 `--side both` 控制两侧；选择单侧时另一侧重发实测开度。

新增 console_scripts 入口后，即使已有 symlink 安装，也需要由用户同步后在
机器人 Humble 容器中重新构建一次以生成新命令（依赖已构建并加载时）：

```bash
colcon build --base-paths src --packages-select aimdk_test --symlink-install
source install/setup.bash

# 默认只读，不发送命令。
ros2 run aimdk_test gripper_close

# 闭合两侧夹爪。
ros2 run aimdk_test gripper_close --opening 0 --execute

# 完全张开两侧夹爪。
ros2 run aimdk_test gripper_close --opening 1 --execute

# 只闭合右侧，左侧保持记录的开度。
ros2 run aimdk_test gripper_close --side right --opening 0 --execute

# 只将右侧开度设为 0.5。
ros2 run aimdk_test gripper_close --side right --opening 0.5 --execute
```

参数 `--timeout` 默认 5 秒，约束启动服务发现/响应、DDS 订阅匹配和夹爪到位等待。
MC、手臂和夹爪的反馈时限均默认 0.5 秒，可用对应的三个 `--*-feedback-timeout`
参数调整；观察和执行阶段使用相同配置。
命令以 50 Hz 发送完整双臂位置和两爪开度，不创建肩关节轨迹，直接发送指定开度。
双臂保持容差为 ±3°，两爪目标开度容差为 0.02，并要求收到发布后的新源反馈、
夹爪静止及连续 0.3 秒到位。已处于目标开度的夹爪也会通过新反馈确认。
堵转、物体掉落或硬件故障仍中止；这是位置控制，不是夹持力保持命令。
模式变化、反馈失效及 80 ms 发布间隔保护也与肩方向测试共用。

成功或失败后停止本脚本发布，不自动改变夹爪目标，也不切换模式；停发后的固件行为
尚未完成专门验证。诊断日志前缀为 `gripper_close_`，记录选择侧别、保持的双臂目标、
夹爪起始/目标开度、实际发布和到位结果。2026-10-24 用户已在机器人容器内验证
右夹爪目标开度 `0.5` 及随后闭合至 `0`；同日稍后的 `dual_arm_control`
完整流程又验证了两侧夹爪同时全开至 `1`、保持、闭合至 `0` 及双臂回程，
记录见 [test.md](../../test.md)，用户已确认两侧同时开闭及全开成功。
这不将独立 `gripper_close --side left` 的单侧流程或物体夹持、堵转、掉落等行为
一并记为已验证；夹持力保持和停止发布后的固件行为仍需分别确认。

新增脚本放入 `aimdk_test/`，在 `setup.py` 的 `console_scripts` 中注册
`命令名 = aimdk_test.模块名:main`，并在 `package.xml` 声明新增 ROS 依赖。
本包内模块互相导入时使用 `aimdk_test` 前缀。

维护者邮箱和许可证目前为占位信息，正式发布前需替换为实际值。
