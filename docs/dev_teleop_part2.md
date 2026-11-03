# PICO接入与双臂逐帧IK实施计划

更新日期：2026-11-03。适用PICO 4 Ultra、X2 Ultra、SDK 1.1、ROS 2 Humble、官方模型v1.3.0。
状态：已完成机器人容器内PICO只读接收、常驻监听及Reconnect恢复验证，可进入真实输入映射与双侧IK联调。
映射、双侧逐帧IK及回放代码已实现，真实PICO到IK链路待验证；持续执行与补偿待接入。
项目部署流程见[根目录说明](../README.md)，既有接口见[功能包说明](../overlay/src/functions/README.md)，
前期依据见[项目调查](archive/x2_teleoperation_project_survey.md)及[启动保持调查](archive/upper_body_startup_hold_investigation_20261030.md)。

## 已验证进展

2026-11-02用户在Orin开发容器运行，下载JSONL后在本地分析。已验证范围如下：

| 项目 | 结果 |
| --- | --- |
| 接收链路 | PICO APK → Orin PC Service → C ABI回调 → `remote_control`完整双手柄帧与JSONL已打通，设备标识为`TestDevice` |
| 常驻监听 | `xr_watch`可先启动等待，再唤醒头显、在App选择Reconnect并开启Controller／Send，原进程开始接收；退出重开App也曾恢复接收 |
| 手柄输入 | 双侧位姿有变化，Grip与Trigger均覆盖0～1，摇杆双轴有变化；按钮按下尚未记录到 |
| 接收节奏 | 最新会话4286帧、47.68秒输入段，约89.88Hz；接收间隔P50/P95/P99为11.01/18.93/27.38ms，最大74.66ms；消费期间最大积压1条 |
| 日志质量 | 两次会话均无回调接收序号缺口，源时间无缺失、重复或回退；Ctrl-C后有`exit: interrupted`记录 |

依据：[首次恢复会话](../log/xr_watch_20261102T124809_360905Z_795c688e.jsonl)、
[Reconnect会话](../log/xr_watch_20261102T125615_529097Z_9eb5a8e8.jsonl)。
帧率按首末完整帧的接收单调时间计算；上述结果仅覆盖输入接收，不代表IK或运动发布频率。

## 部署与依赖

首轮在Orin机器人开发容器部署XR接收与控制，头显和Orin接入同一局域网。
出现资源竞争并影响控制调度时，将PC Service及XR输入适配迁至PC，通过网络向Orin传输完整XR帧；
坐标映射、持续会话、IK与AimDK执行保留在Orin。

| 位置 | 组件 | 部署要求 |
| --- | --- | --- |
| PICO 4 Ultra | [XRoboToolkit-Unity-Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client)发布版APK | 开启手柄数据发送，连接Orin服务地址；记录APK及PICO OS版本 |
| Orin容器 | [XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0) ARM64 | 已安装`headless_1.0.0.0_arm64.deb`，包名`roboticsservice`、版本1.0.0.0，携带Qt 6.6.2；补装`xdg-utils`后配置成功 |
| Orin容器 | 服务随包PXREARobotSDK | 已使用`/opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so`接收完整回调，日志保存库路径及SHA256 |
| Orin容器 | remote_control | XR帧、SDK接入、Grip会话、坐标映射、输入记录与统计；映射参数显式配置 |
| Orin容器 | ROS 2 Humble、AimDK 1.1、aimdk_msgs、aimdk_functions、NumPy | 沿用现有underlay与overlay，Python绑定使用兼容rclpy的解释器 |
| 开发电脑 | ADB | 安装头显APK |

当前使用Python 3.10.12。服务入口为`/opt/apps/roboticsservice/runService.sh`，
头显连接机器人局域网地址；SDK连接本机`127.0.0.1:60061`。
常驻命令、配置与日志路径见[输入测试说明](../overlay/src/remote_control/README.md#常驻输入测试)。

## 数据流与关键接口

首轮数据流截至双侧统一提交及只读日志；持续发布和补偿按后续步骤接入。

```text
PICO双手柄 → 头显XRoboToolkit APK → 局域网 → Orin PC Service
  → PXREARobotSDK回调 → Python完整XR帧 → 输入映射与持续会话
  → torso_link下左右TCP目标 → 双侧ArmKinematics.track()
  → 双侧统一提交 → 执行层校验及可选补偿
  → UpperBodyCommandArray → /mc/upper_body_command → 原厂运控

MC的HAL手臂位置命令 → UpperBodyFeedbackPort → 首次接管参考及日志
原始双臂／夹爪／模式反馈 → UpperBodyFeedbackPort → 执行校验及日志
骨盆IMU／腰反馈 → 启用补偿时的计算与日志
```

| 接口 | 数据与实现要求 |
| --- | --- |
| SDK回调 → XR输入帧 | 在同一回调内提取左右位姿、Grip、扳机、按钮、源时间及可用的跟踪状态；附加接收序号、单调接收时间、连接状态，明确字段缺失的处理 |
| XR输入帧 → TCP目标帧 | 核对坐标轴、左右手系、四元数顺序和单位；输出torso_link下左右位置m、完整姿态、每侧使能状态及关联序号和时间 |
| TCP目标帧 → IK结果 | 每侧调用包内ArmKinematics.track()，输出7关节rad、SEW、分支、残差和接受／失败原因 |
| IK结果 → 执行目标 | 两侧均通过后统一提交状态与完整14关节目标；夹爪输入映射为原生接口要求的目标 |
| 执行目标 → SDK消息 | 复用build_upper_body_command()及原生上肢发布路径，扩展UpperBodyExecutor的在线目标消费与持续发布 |
| SDK反馈及MC命令 → 控制器 | 复用UpperBodyFeedbackPort、按名称解码、源时间推进检查、接收时效及错误锁存；MC的HAL位置命令用于首次接管参考 |

通过SDK C ABI回调复制完整数据，提供快照及新帧消费接口；保留原始接收序号与适配采样映射。
记录源时间的实际生成方式和精度，跨设备延迟测量先校准时钟。
XR输入、会话、坐标映射及输入日志放入remote_control；双侧IK与机器人执行适配放入aimdk_functions。
两包分别注册命令；aimdk_functions声明remote_control依赖。

## TCP与求解

采用参考x2ik 2.1.0、提交444598038cf0e5b27e32e3e5295ab24503ecd43f的omnipicker名义TCP。
左右ArmKinematics均使用tcp_mode='gripper'，从包内vendor/assets/tcp_presets.json加载完整工具变换。
FK、IK、参考建立与日志统一使用该配置；启动记录工具名、左右变换、配置哈希及实际负载。
实体工具平移、方向和抓取精度单独测量。

每个提交控制器的新目标帧执行以下步骤：

1. 首次Grip、同会话重按及中断恢复的参考建立按[功能包说明](../overlay/src/functions/README.md#现场只读ik与统计)执行；分别保存两侧上一已接受理想关节、SEW及解析分支。
2. 对使能侧按目标位姿跟踪求解；未使能侧保持最后已接受目标，首次使用采样目标。仅映射平移时保留接管时TCP姿态。
3. 沿用有限性、限位及既有10°实验范围检查，分别记录相邻帧变化与相对会话起点总变化；禁用可达域投影、全局兜底及放宽姿态约束。
4. 两侧候选均通过后统一提交关节、SEW和分支；任一侧失败，两侧均不推进，执行异常沿用停止发布并记录原因的契约。
5. 理想IK目标与补偿后发送目标分别校验和记录，下一帧种子使用理想关节。

## 持续会话与调度

首轮入口只读，补偿关闭，拒绝--execute。后续开放执行时沿用URS实际模式及RUNNING确认。
Grip按下建立手柄与机器人TCP参考，按相对位移和姿态增量映射目标；松开保留最后目标，重按重新建立参考。
持续会话复用进程与发布器。断链、跟踪丢失或重定位时记录异常并停止发布，恢复时重新建立参考。
现场记录停发后的实体保持与超时行为。

首次接入及中断恢复重新采样HAL命令；当前只读入口中断后由操作者重启，恢复初帧接受条件仍待确定。
补偿偏置策略在执行接入时确定，手柄每帧更新或Grip重按保持发布会话与补偿状态独立。
真机持续会话前完成补偿接入方案、只读回放审阅及启动下沉评估。

当前实测输入约90Hz；每个提交控制器的新目标帧产生一次双侧求解结果或明确失败记录。
重复缓存读取跳过IK，输入适配采样、超时、积压与拒绝均记录。
首轮命令发布及有效输入下的补偿计算保持50Hz；等待新目标期间复用最后已接受理想目标，输入中断按会话异常处理。
隔离IK对20ms发布调度的阻塞，具体执行上下文由周期测量确定。
完整记录输入、映射、IK、补偿、发送和反馈的序号与时间关联。

## 后续实施顺序

代码在本地修改和静态检查，构建、运行与测试由用户同步后在机器人开发容器执行。
现有只读入口及配置见[remote_control](../overlay/src/remote_control/README.md)与
[功能包遥操入口](../overlay/src/functions/README.md#pico双臂只读回放)。
下一步先完成输入恢复约定与实际映射配置，再复用现有回放、观测和统计入口。
两包状态契约用例的机器人容器执行结果仍需记录。

| 步骤 | 操作 | 预期结果 |
| --- | --- | --- |
| 1 输入约定与恢复 | 核对轴向、单位、姿态；确定停流、恢复初帧及重定位处理；填写输入与映射配置 | 配置与现场动作一致，恢复后的参考建立条件明确；待定项见下节 |
| 2 真实输入回放 | 用已录XR帧、含HAL命令的快照和gripper TCP运行`upper_body_teleop_replay`，覆盖左右平移、旋转、组合及单侧失败 | 每个提交目标有结果，任一侧失败时两侧状态不推进；停流及恢复片段按已确认约定处理 |
| 3 实时只读联调 | 用`upper_body_teleop_observe`接入PICO与机器人反馈，验证Grip参考、松开／重按及夹爪目标映射 | 手柄动作与TCP目标一致，输入至双侧IK结果可追溯，保持只读、补偿关闭 |
| 4 周期与负载 | 按实测输入节奏验证双侧IK、反馈、补偿、日志和50Hz隔离话题发布；明确约90Hz输入的逐帧消费或采样策略 | 记录输入至目标序号映射、耗时、发布／接收间隔及积压，确认完整调度可用 |
| 5 小幅执行与恢复 | 确认补偿接入和启动保持方案后，验证零目标保持、双臂小幅动作、Grip重接管、夹爪及输入中断恢复 | 单个持续发布会话可追溯；停发后的实体响应、首次下沉和重接管偏移有记录 |
| 6 轨迹录制与播放 | 录制、读回并播放双臂与夹爪轨迹 | 输入、理想目标、实际发送、反馈和夹爪动作可关联，完成情况及偏差有记录 |

周期测量分别报告输入、双侧IK及发布侧的P50/P95/P99/最大值、超期次数与积压趋势。
测量字段及方法见[频率与周期验证](archive/pico_rate_and_online_cycle_20261030.md)。

## 注意事项与待验证项

- **停流与连接状态**：首次会话有6.05秒停流，Reconnect会话退出前约6.75秒无新输入，均未收到设备离线或Service断开事件。
  用户观察到头显睡眠会中断输入；睡眠、关闭Send和网络中断需分别记录操作时间复验。
  现有只读IK入口已有显式`input-timeout`，需结合实测确认取值及异常处理；SDK连接状态不能单独作为输入有效依据。
- **恢复初帧与跟踪**：两次恢复后分别出现4帧、2帧双侧零位置和单位旋转，Controller未提供跟踪有效性，`tracking_valid`均为未知。
  恢复初帧的接受条件待确定；零位姿不直接定义为通用失效判据。只读映射的未知跟踪策略须显式配置。
- **参考空间**：现有配置沿用`pico-session-001`，重连前后坐标连续性尚未验证。
  按输入包约定在重定位后更新`reference_space_id`；补测Reconnect是否重置参考空间，并验证松开Grip后重新按下建立参考。
- **映射与输入覆盖**：现用`xyzw`、位置缩放1.0；实际轴向、手系、距离与旋转方向仍待现场核对。
  确定位移倍率、姿态模式、Grip阈值及夹爪映射，补测按钮按下与摇杆点击。
- **版本与时间**：补记APK及PICO OS版本，核对源时间生成方式与精度；未校准时钟前不计算跨设备延迟。
  两次均为Ctrl-C收尾，`recording_complete=false`需结合`interrupted`解释；固定时长正常完成及Service重启后的SDK重连另行验证。
- **执行接管**：确定首次接管与中断恢复的补偿策略，核实URS首次命令保持语义；保持只读直至完成回放、实时映射和周期验证。
