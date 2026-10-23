# 上半身测试接口核实记录

核实范围：当前工作空间 SDK 1.1.0 的消息源码、预编译 Python 消息、Python 示例及本地 HTML 文档。机器人为用户确认的 X2 Ultra / 夹爪 / 固件 1.1.5；本记录没有连接机器人，没有初始化 ROS 节点，没有做固件 1.1.5 真机验证。资料未明确的行为保持为未知。

> 当前 head_test 行为更新：按用户要求，不再检查 FSM。启动通过 GetMcAction 确认 STAND_DEFAULT / RUNNING，执行期间检查 HEAD_ONLY / RUNNING，保留 body=STAND 与反馈检查。下文 FSM 编码冲突为接口核实记录，不再作为该脚本的执行门槛；raw=4 的含义仍未核定。

## 关键结论

1. X2 Ultra 原旗舰版首测只开放头部 yaw。机型活动范围表明确原旗舰 yaw 为 ±20°、pitch 为“无”；固定两元素消息不意味着两关节均可运动。
2. 原生运控继续运行，控制使用 `/mc/upper_body_command`，不发布 HAL 关节命令，不停止 MC。
3. **FSM 编码存在严重资料冲突**：消息源码和预编译 Python 为 `STABLE=2, SAFE=4`；本地运控状态文档为 `STABLE=4, SAFE=6`。不可直接把数值 4 当成稳定。只读阶段必须记录实际数值和本地消息常量；在厂家/固件核实之前不能声称某个数值已验证安全。
4. 上肢停止发布后的超时、保持和交接行为未见明确说明；不能把速度控制输入源的“超时清零”或 HAL 命令的“无超时”直接移植到该接口。

## 指令与模式

依据：[上肢章节](cn/_build/html/zh-cn/dev/Interface/control_mod/upper_body_control.html)、[消息定义](../src/aimdk_msgs/interface/robot/mc/openapi/msg/UpperBodyCommandArray.msg)、[Python 示例](../src/py_examples/py_examples/upper_body_control.py)。

| 项目 | 已核实内容 |
|---|---|
| 指令 | `/mc/upper_body_command`，`UpperBodyCommandArray` |
| QoS / 频率 | `BEST_EFFORT + VOLATILE`，推荐 50 Hz |
| 头部 | `head_pos=[yaw,pitch]`，rad；对应 `head_yaw_joint,head_pitch_joint` |
| 双臂 | `arm_pos` 固定 14 元素，左 7 后右 7；顺序为 shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll |
| source | 专门章节明确“任何非空字符串”；应使用独立二开名字，避免系统预置源名称 |
| 时间戳 | 非零 `header.stamp` 与运控接收时间差不得超过 200 ms；全零时运控使用接收时刻。零命令戳不解除反馈新鲜度检查 |
| 头部模式 | 先 `STAND_DEFAULT`，再 `HEAD_ONLY` |
| 手臂模式 | 先 `STAND_DEFAULT`，再 `UPPERBODY_REMOTE_SPLIT` |

上肢专门章节明确 URS 下头部目标无效；模式总表和设置模式示例描述“头+双臂+手”与之冲突。首版按专门章节采用独立头部模式。

现有 Python 示例的 `head` 配置为 `hand_sub_mode=0, hand_pos=[]`，双臂数组填零；这是 HEAD_ONLY 例程的直接依据。消息常量仅定义手部模式 1/2/3，没有定义通用忽略/保持模式 0。**不能推断 URS 下 0 或空数组可保持夹爪**。HEAD_ONLY 之外字段隔离的具体固件实现尚未验证。

[模式章节](cn/_build/html/zh-cn/dev/Interface/control_mod/modeswitch.html)与消息定义：

- Set：`/aimdk_5Fmsgs/srv/SetMcAction`，请求 `header,source,command.action_desc`。响应 `response.status.value==CommonState.SUCCESS(1)`，且 `response.header.code==0` 才视为成功响应。
- Get：`/aimdk_5Fmsgs/srv/GetMcAction`，请求 `request:CommonRequest`；响应 `header.code`、`info.action_desc`、`info.status.value`。
- `McActionStatus` 源码：`IDLE=0,RUNNING=100,TRANSITION=200`。文档另列 300 不可中断切换，源码没有对应常量，软件不能把未知状态当 RUNNING。
- 有界轮询实际模式名及 RUNNING 是测试程序的保守完成判据，避免只凭 Set 回包宣称接管完成；资料没有给出 Set 返回与物理完成之间的精确定时保证。
- 错误包括姿态非法、启动中、运动忙、安全保护禁止、无转换路径；不存在从任意姿态自动站起均可成功的保证。初始要求操作员已站稳，程序只接受已核实状态。
- 模式命令不受输入源优先级保护，但受安全保护约束。因此发生异常时切回 `STAND_DEFAULT` 可能被拒绝，不是可靠急停或必达恢复动作。

## 头部反馈、限位与时效

依据：[关节章节](cn/_build/html/zh-cn/dev/Interface/control_mod/joint_control.html)、[机型范围表](cn/_build/html/zh-cn/dev/about_agibot_X2/joint_name_and_limit.html)、[JointStateArray](../src/aimdk_msgs/interface/robot/hal/msg/JointStateArray.msg)、[JointState](../src/aimdk_msgs/interface/robot/hal/msg/JointState.msg)、[MessageHeader](../src/aimdk_msgs/interface/robot/common/msg/MessageHeader.msg)。

- `/aima/hal/joint/head/state`，`JointStateArray`，发布 QoS `BEST_EFFORT + TRANSIENT_LOCAL`，文档频率 333 Hz。订阅必须兼容，且不能把历史缓存当实时有效反馈。
- 字段 `header.stamp` 世界时间，`header.meas_stamp` 接收/测量时间，`header.sequence` 序号；`state.value` 为关节组异常状态；`joints[]` 含 `name,position,velocity,effort,error_code`。
- `DomainErrorState` 为 NONE=0、DAMPING=1、POWERDOWN=2、DISABLE=3、DISCONNECT=4；测试保守只接受 NONE 且关节 error_code 为 0，有限数值、无重名、必需关节齐全。
- 按名称匹配并重排为 yaw/pitch，禁止假设反馈数组始终有两项或始终按固定顺序。文档定义两槽、原旗舰仅 yaw 有效；实际原旗舰反馈是否含 pitch 占位，需只读阶段观察。若运动实现保守要求两槽，应明确报告这一软件门槛，不能把缺 pitch 误称机器人故障。
- 世界戳年龄和单调接收间隔分别检查；持续收到相同缓存/停止前进的时间戳不能算新鲜。跨机世界时钟偏差、零戳/未来戳需要单独报告，不能用本机接收时刻伪造测量时间。
- X2 Ultra yaw 保证范围 ±20°（约 ±0.349066 rad），pitch 不开放。通用 `motocontrol.py` 给出的 yaw ±0.366 rad 比机型保证范围宽，不应直接复制；测试再留软件余量并校验当前起点和整个轨迹。

`GetAllJointState` 有 `head_joints` 等数组，但 `reponse` 通用响应字段未启用、没有各数组独立采样戳，不适合作为持续反馈时效的替代依据。

## 原生站立状态反馈与冲突

依据：[运控状态章节](cn/_build/html/zh-cn/dev/Interface/control_mod/mc_status.html)、[McCommonState](../src/aimdk_msgs/interface/robot/mc/data/msg/McCommonState.msg)、[McFsmState](../src/aimdk_msgs/interface/robot/mc/data/msg/common/McFsmState.msg)。

- `/aima/mc/common/state`，`McCommonState`，`BEST_EFFORT + TRANSIENT_LOCAL`，10 Hz。
- 可读取 `header,action_info,body_status.value,fsm_state.current_state`；`body_status` 的 STAND=1 在源码和文档一致。它是运控分类状态，不等于物理稳定性保证，仍需现场观察。
- FSM 源码及 `install/.../_mc_fsm_state.py`、`prebuilt_x86_64/.../_mc_fsm_state.py` 一致：UNKNOWN=0,STARTING=1,STABLE=2,MOVING=3,SAFE=4,SPECIAL=5,TEST=6。
- HTML 文档则列 UNKNOWN=0,PREPARE=1,START=2,FINISH=3,STABLE=4,MOVING=5,SAFE=6,SPECIAL=7,TEST=8。须作为明确待核实项，不得悄悄合并两套数值或同时接受 2/4 为安全。
- HEAD_ONLY 运行期间 FSM 是否持续报告稳定、切换时如何变化，资料未保证，必须真机只读/厂家核实。模式名 RUNNING 不能代替新鲜站立状态。

## 输入源机制

[MC 控制章节](cn/_build/html/zh-cn/dev/Interface/control_mod/MC_control.html)说明连续速度/姿态控制需要注册和启用自定义输入源；上肢专门章节只要求非空 source，Python 上肢例程也没有注册步骤。运控状态章节明确 `input_source` 仅反映速度/姿态控制权。

因此，有依据的是“非空独立 source，禁止冒用系统源”；未知的是上肢固件内部是否使用同一仲裁/超时链路。不能以 `GetCurrentInputSource` 变化或不变化证明上肢已被接受，也不能凭通用仲裁章节自动增加高优先级注册。首版记录这一未知，通过模式和实际头部跟踪验收；不修改系统预置源。

## 夹爪保持方案依据

依据：[末端章节](cn/_build/html/zh-cn/dev/Interface/control_mod/endeffector.html)、[HandStateArray](../src/aimdk_msgs/interface/robot/hal/msg/HandStateArray.msg)。

- 反馈 `/aima/hal/joint/hand/state`，`HandStateArray`，`BEST_EFFORT + TRANSIENT_LOCAL`，约 200 Hz，负载大时会降频。
- 两侧 `*_hand_type.value` 必须为 CLAW=2，每侧 `*_hands` 长度 1。NONE=0/ERROR=255 时数组会保留上次有效值，不能使用旧 position。
- 夹爪 `HandState.position` 是 0..1 当前行程；命令 position 同范围，1 为全开；上肢 `hand_pos=[left_open,right_open]` 为 0..1。因此，将两侧切换后稳定有效的当前 position 填入模式 1 目标，是有文档语义依据的“保持当前开度”实现方案。
- 这仍是重发位置目标，并非硬件专门 hold 指令或力保持保证。选择空载且静止（state=0）、faultcode=0、新鲜有效反馈作为前提；state=1移动、2堵转、3物体掉落不作为普通保持起点。
- 未知：URS 刚接管而首帧尚未到达时的夹爪目标，目标回发的内部默认速度/力矩，以及接管是否先改变开度。单臂阶段须核实，不能保证“完全不主动开合”。
- 不使用底层手部控制路径：文档要求停止原生 MC，与此次原生站立目标冲突。

## 停止、退出与仍需验收事项

正常结束的实现策略为：有效反馈下平滑回测试起点，确认达到目标，再请求并确认默认站立。它是软件流程设计，默认站立接管时的头臂姿态和插值不由本 SDK 文档保证。

异常发生后停止推进轨迹；无有效反馈时不盲目返程；允许在通信仍有效、状态允许时有界尝试交接默认站立并报告结果，不能声称失败后机器人必定安全。停止发布不是已证实的保持/急停。HAL `/aima/hal/joint/*/command` 文档的无超时警告仅针对 HAL，不代表上肢接口有或没有超时保护。

下一步联合验收应先只读，记录模式、原旗舰实际头部槽位、反馈时戳/时钟、FSM 原始值及官方编码核定；再由用户明确启动一次 yaw 小幅运动。单臂开发前另核实夹爪接管连续性。未以代码离线检查替代以上真机事项。
