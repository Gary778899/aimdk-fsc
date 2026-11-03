# remote_control

PICO/XR输入快照、相对位姿映射和Grip会话。纯Python核心依赖NumPy，与ROS消息和AimDK解耦。
机器人FK/IK与反馈适配位于`aimdk_functions`。首轮只读，补偿关闭，真机执行尚未开放。

## 常驻输入测试

`xr_watch`用于Service已启动后的只读接入测试。默认监听已确认的设备`TestDevice`，
读取`/home/agi/aimdk-fsc/data/pico_config/input.json`，使用已安装ARM64服务随包SDK。
该配置须已按[输入配置说明](SDK_INTERFACE.md#输入配置)填写；缺失或非法时启动失败。
实现状态：本地静态检查，待机器人容器构建与验证。

同步后在机器人Humble容器首次构建新入口，再启动：

```bash
cd /home/agi/aimdk-fsc
bash scripts/build_overlay.sh remote_control
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run remote_control xr_watch
```

启动后持续等待；头显打开XRoboToolkit，确认连接、Controller及Send开启。
终端立即显示服务与设备事件，每秒输出本窗口帧数、接收帧率、累计帧数、
最近完整帧接收年龄、队列积压和左右手柄的位置m、旋转矩阵、Grip、Trigger、按钮与摇杆。
窗口无新完整帧时仅显示等待状态，不重复显示历史手柄快照。
`receive_hz`按本显示窗口完整帧的接收单调时间计算`(N-1)/(t_last-t_first)`，不足两帧为null。
显示节奏与输入接收节奏独立，全部回调和完整帧按FIFO写入日志。

默认每次启动创建`/home/agi/aimdk-fsc/log/xr_watch_<UTC时间>_<随机标识>.jsonl`，
从启动起保存原始回调，包含尚无头显时的服务连接事件；收到所选设备的完整帧后保存标准XR帧。
设备离线或Service断开时清除显示缓存、记录失联并继续等待SDK后续事件。
头显重新连接并发送时恢复显示与记录；命令不启动Service，Service重启后的SDK自动重连待现场验证。
Send关闭后若SDK未报告离线，显示零新帧及持续增加的接收年龄，设备状态仍保留最近实际事件。

Ctrl-C结束、关闭SDK并排空原始回调，退出码130，日志结束原因为`interrupted`。
可使用`--duration 60`限定总监听时长（包含等待），无完整帧返回1。
`--device-id`、`--input-config`、`--sdk-library`、`--log-dir`可覆盖上述默认值，
`--display-interval`设置大于0的显示间隔秒（默认1）。
在新参考空间或应用重定位后，结束监听并更新配置中的`reference_space_id`再启动。

日志可用`xr_stats <日志路径>`统计；Ctrl-C收尾或包含失联的文件会得到`recording_complete=false`。
该结果应结合退出原因与异常记录解释。`read_frames()`遇到失联仍停止，不能跨断链直接回放。
正式的固定时长录制使用[SDK接口说明中的xr_record](SDK_INTERFACE.md#命令与日志)。

验收时依次验证：仅Service运行时持续等待；头显发送后左右位姿和按键随操作变化；
关闭Send后显示无新帧；退出并重连头显后恢复输入；Ctrl-C后日志可读且有exit记录。

## 接口

- `frames.py`：不可变`Pose`、`Controller`、`XRFrame`、`TCPGoal`、`TargetFrame`；提供`to_dict/from_dict`。
- `geometry.py`：四元数顺序显式指定为`xyzw`或`wxyz`，转换后存储旋转矩阵；位置单位m。
- SDK原始回调、输入配置、录制与统计命令见[SDK接口说明](SDK_INTERFACE.md)。
- `mapping.py`：`MappingConfig.load(path)`加载配置；`Session.propose(frame, robot_tcp_poses, hand_openings)`生成双侧候选。
- 双侧IK成功后预检`Session.validate_commit(proposal)`，再统一提交IK和`Session.commit(proposal)`；失败调用`reject(proposal)`。
- 每侧Grip上升沿使用调用方传入的机器人实际反馈FK建立参考；连续映射使用原参考，Grip松开保持已提交目标。
- 断链、已报告的跟踪失效、设备／参考空间变化或源时间回退清除参考；先观察Grip松开，再重新按下接管。

`XRFrame.seq`为适配接收序号；`source_time_ns`保留源时间，`received_monotonic_s`为本机接收时间。
`tracking_valid=None`表示未提供，按显式`unknown_tracking`策略处理。
`reference_space_id=None`表示未提供参考空间标识；设备接入时核对重定位事件来源。

## 映射配置

复制`config/mapping.example.json`并填写所有待定字段。模板中的null占位值不能直接用于接入。

| 字段 | 定义 |
| --- | --- |
| `basis` | 输入坐标至机器人基坐标的3×3正交矩阵，允许反射 |
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

## 验证

按根目录流程在机器人容器构建后运行：

```bash
python3 -m unittest discover -s src/remote_control/test
```

覆盖坐标反射、Grip重接管、拒绝回滚、跟踪／连接失效、源时间回退、夹爪保持及帧读回。
实际输入字段、轴向、PICO新帧率和容器内周期验证见[实施计划](../../../docs/dev_teleop_part2.md)。
