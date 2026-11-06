# remote_control

PICO/XR输入快照、相对位姿映射和Grip会话。纯Python核心依赖NumPy，与ROS消息和AimDK解耦。
机器人FK/IK与反馈适配位于`aimdk_functions`。本包保持只读；机器人执行入口及约定见
[遥操持续执行](../functions/README.md#遥操持续执行)。

## 常驻输入测试

`xr_watch`自动启动或复用本机XRoboToolkit Service，再进行只读接入测试。默认监听已确认的设备`TestDevice`，
读取`/home/agi/aimdk-fsc/src/remote_control/config/input.json`，使用已安装ARM64服务随包SDK。
配置源文件为本地`overlay/src/remote_control/config/input.json`，随现有同步任务推送，无需单独创建。
当前固定值及重定位后的更新约定见[输入配置说明](SDK_INTERFACE.md#输入配置)。
实现状态：本地静态检查，待机器人容器构建与验证。

同步后在机器人Humble容器构建更新入口，再启动：

```bash
cd /home/agi/aimdk-fsc
bash scripts/build_overlay.sh remote_control
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run remote_control xr_watch
```

启动时检查`127.0.0.1:60061`：端口已监听则复用；否则调用
`bash /opt/apps/roboticsservice/runService.sh`，等待端口监听后初始化SDK。
默认启动等待15秒，脚本返回0后继续等待后台服务；非零退出或端口等待超时则报错退出。
该时限可用`--service-start-timeout`调整。
端口可连接仅用于启动协调，实际服务连接以SDK回调为准。
自启服务的标准输出及错误保存在与JSONL同名的`.service.log`文件中。
退出时先关闭SDK并收尾录制。当前厂商程序会转入后台，后台服务及复用的服务保持运行，下一次启动检查端口并复用。
若替换的启动脚本仍在前台运行，则向其进程组发送SIGTERM，5秒未结束时发送SIGKILL。

服务就绪后持续等待；头显打开XRoboToolkit，连接机器人局域网地址，确认Controller及Send开启。
终端立即显示服务与设备事件，每秒输出本窗口帧数、接收帧率、累计帧数、
最近完整帧接收年龄、队列积压和左右手柄的位置m、旋转矩阵、Grip、Trigger、按钮与摇杆。
窗口无新完整帧时仅显示等待状态，不重复显示历史手柄快照。
`receive_hz`按本显示窗口完整帧的接收单调时间计算`(N-1)/(t_last-t_first)`，不足两帧为null。
显示节奏与输入接收节奏独立，全部回调和完整帧按FIFO写入日志。

默认每次启动创建`/home/agi/aimdk-fsc/log/xr_watch_<UTC时间>_<随机标识>.jsonl`，
从启动起保存原始回调，包含尚无头显时的服务连接事件；收到所选设备的完整帧后保存标准XR帧。
设备离线或Service断开时清除显示缓存、记录失联并继续等待SDK后续事件。
头显重新连接并发送时恢复显示与记录；运行中不自动重启Service，Service重启后的SDK自动重连待现场验证。
Send关闭后若SDK未报告离线，显示零新帧及持续增加的接收年龄，设备状态仍保留最近实际事件。

Ctrl-C结束、关闭SDK并排空原始回调，退出码130，日志结束原因为`interrupted`。
可使用`--duration 60`限定服务就绪后的监听时长（包含等待头显），无完整帧返回1。
`--device-id`、`--input-config`、`--sdk-library`、`--log-dir`可覆盖上述默认值，
`--display-interval`设置大于0的显示间隔秒（默认1）。
`--service-script`可覆盖服务启动脚本路径。
在新参考空间或应用重定位后，结束监听并更新配置中的`reference_space_id`再启动。

日志可用`xr_stats <日志路径>`统计；Ctrl-C收尾或包含失联的文件会得到`recording_complete=false`。
该结果应结合退出原因与异常记录解释。`read_frames()`遇到失联仍停止，不能跨断链直接回放。
正式的固定时长录制使用[SDK接口说明中的xr_record](SDK_INTERFACE.md#命令与日志)。

验收时依次验证：未启动Service时单独运行命令，服务自动启动并等待头显；
头显发送后左右位姿和按键随操作变化；关闭Send后显示无新帧；退出并重连头显后恢复输入；
Ctrl-C后日志可读且有exit记录；后台服务保持运行，重跑命令复用已有服务；
预先手动启动Service后重跑命令，退出时该服务仍运行。

## 接口

- `frames.py`：不可变`Pose`、`Controller`、`XRFrame`、`TCPGoal`、`TargetFrame`；提供`to_dict/from_dict`。
- `geometry.py`：四元数顺序显式指定为`xyzw`或`wxyz`，转换后存储旋转矩阵；位置单位m。
- SDK原始回调、输入配置、录制与统计命令见[SDK接口说明](SDK_INTERFACE.md)。
- `mapping.py`：`MappingConfig.load(path)`加载配置；`Session.propose(frame, robot_tcp_poses, hand_openings)`生成双侧候选。
- `Proposal.report`保存每侧Grip事件、输入／TCP增量与参考位姿；`Session.describe()`返回已提交参考及输入消费状态，供失败前后对照。
- 双侧IK成功后预检`Session.validate_commit(proposal)`，再统一提交IK和`Session.commit(proposal)`；失败调用`reject(proposal)`。
- 执行侧接受过渡目标时使用`Session.commit(proposal, accepted_target=过渡目标)`，仅更新保持位姿与夹爪开度，Grip参考延续本帧候选。恢复与失败处理策略见[功能包说明](../functions/README.md#ik无解保持与恢复)。
- 执行侧取消尚未完成的关节推进时，在候选已提交或拒绝后调用`Session.hold(当前发布理想位姿, 开度)`更新保持值；Grip参考延续。发布侧行为见[渐进规则](../functions/README.md#发布侧关节渐进靠近)。
- 每侧Grip上升沿使用调用方传入的机器人TCP建立参考；连续映射使用原参考，Grip松开保持已提交目标。
  X2的首次HAL命令参考、同会话理想目标参考及中断恢复见[功能包说明](../functions/README.md#现场只读ik与统计)。
- 断链、已报告的跟踪失效、设备／参考空间变化或源时间回退清除参考；先观察Grip松开，再重新按下接管。

`XRFrame.seq`为适配接收序号；`source_time_ns`保留源时间，`received_monotonic_s`为本机接收时间。
`tracking_valid=None`表示未提供，按显式`unknown_tracking`策略处理。
`reference_space_id=None`表示未提供参考空间标识；设备接入时核对重定位事件来源。

## 映射配置

首轮只读联调使用[config/mapping.json](config/mapping.json)，随现有同步任务推送，
机器人源码路径为`/home/agi/aimdk-fsc/src/remote_control/config/mapping.json`，用`--mapping-config`指定。
配置采用[XRoboToolkit示例的坐标转换矩阵](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/blob/main/xrobotoolkit_teleop/utils/geometry.py)、
1.0位移倍率、相对姿态、0.9 Grip阈值、Trigger控制夹爪和允许未知跟踪状态。
现场夹爪配置为`gripper_mode=trigger`、`trigger_openings=[1.0, 0.0]`：按住Grip后，
普通位姿帧将Trigger从0到1线性映射为夹爪开度从1到0；松开Grip保持最后已接受开度。
Grip边沿帧保持开度，随后首个普通帧开始应用Trigger目标。
现场配置使用`reference_mode=grip_head_yaw`：每侧Grip按下时，从同帧头显姿态提取水平朝向，
将该朝向固定为本侧本次Grip的前方；手柄相对平移与旋转均按此方向映射到机器人。
头部俯仰、横滚和平移不参与方向标定；保持Grip期间转头不改变目标，松开保持目标，重按重新标定。
左右侧独立保存朝向；参考与映射候选联合提交，拒绝候选时保留上一已接受参考。
设固定轴转换为`B`、Grip时转换后的头显水平旋转为`Rz`，本次有效映射为`A=Rz.T @ B`；
平移为`scale * A @ (p-p0)`，相对旋转为`A @ (R @ R0.T) @ A.T`，再作用于机器人Grip参考。
须在XRoboToolkit同时开启Head和Controller tracking；该模式收到缺少Head的完整帧时明确报错。
初次标定时头显前向的水平投影须非零；实际轴向及旋转方向待现场核对。
`reference_mode=world`保留固定空间方向，缺省字段时使用此模式，供历史日志和既有配置回放。
`setup.py`同时将该文件安装到`share/remote_control/config/`。
其他配置可复制`config/mapping.example.json`填写；模板中的未填写null占位值不能直接用于接入。

| 字段 | 定义 |
| --- | --- |
| `basis` | 输入坐标至机器人基坐标的3×3正交矩阵，允许反射 |
| `reference_mode` | `grip_head_yaw`在每侧Grip建立参考时固定头显水平朝向；`world`使用固定`basis`，缺省为`world` |
| `translation_scale` | 大于0的位移比例 |
| `rotation_mode` | `relative`映射姿态增量；`fixed`保持接管时TCP姿态 |
| `grip_threshold` | `(0,1]`，Grip大于等于该值时按下 |
| `gripper_mode` | `hold`保留开度；`trigger`按扳机线性映射 |
| `trigger_openings` | trigger模式为扳机0、1对应的两项开度；hold模式为null |
| `unknown_tracking` | `reject`拒绝未知有效性；`allow`允许记录和只读映射未知有效性输入 |
| `base_frame` | 输出TCP目标父坐标系；X2适配使用`torso_link` |

SDK解码的四元数顺序与输入单位在输入配置中填写；本配置的`translation_scale`只表示操作者位移倍率。
映射公式为`p_target = p_robot_ref + scale · B · (p_xr - p_xr_ref)`；
姿态增量为`R_target = B · R_xr · R_xr_refᵀ · Bᵀ · R_robot_ref`。

`Session.observe_input(frame)`可由独立接收会话逐帧校验连接、时间、跟踪及参考空间。
`Session.propose(..., hold_motion=True)`只更新Grip参考或保持状态，位姿与夹爪保持已接受值。
现场20Hz选帧及边沿调度由[功能包](../functions/README.md#现场只读ik与统计)定义。

## 验证

按根目录流程在机器人容器构建后运行：

```bash
python3 -m unittest discover -s src/remote_control/test
```

覆盖坐标反射、Grip重接管、拒绝回滚、跟踪／连接失效、源时间回退、夹爪保持及帧读回。
实际输入字段、轴向、PICO新帧率和容器内周期验证见[实施计划](../../../docs/dev_teleop_part2.md)。
