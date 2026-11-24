# remote_control

本包负责 XR Service 接入、PICO SDK 回调封装、Tracking 解析、输入失效处理与 XR 日志。Grip 接管时，映射器使用调用方提供的双臂 TCP 参考位姿和夹爪开度，将手柄相对运动转换为候选 TCP 目标。输入与映射核心为纯 Python，可离线复用。

现场由 `aimdk_functions` 调用映射器，完成机器人 FK／IK、目标接受或保持及执行。录制时 `data_collection` 订阅原始 XR 话题，本包提供信封解释与 XR 输入质量核验。见[遥操持续执行](../functions/README.md#遥操持续执行)和[单次示教录制](../data_collection/README.md#单次示教录制与读回)。

## 常用命令

以下命令在机器人 Humble 容器中执行。首次部署或包安装内容变更后，先按项目流程同步，再构建 `remote_control`。每个运行终端加载 underlay 和 overlay。 [项目构建与运行环境](../../../README.md#2-容器内构建与验证)

```bash
cd /home/agi/aimdk-fsc
bash scripts/build_overlay.sh remote_control  # 首次部署或包安装内容变更时执行
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
```

| 要做的操作 | 入口 |
| --- | --- |
| 启动或复用 XR Service，现场检查 PICO | [`xr_watch`](#常驻输入测试) |
| 定时保存 XR JSONL，可同时发布原始 ROS 话题 | [`xr_record`](#固定时长录制与发布) |
| 读回 XR JSONL 并统计输入质量 | [`xr_stats`](#日志读回与统计) |
| 进入 URS／RUNNING 后执行机器人双臂遥操 | [查询与切换 URS](../functions/README.md#urs-模式控制) → [启动双臂遥操](../functions/README.md#独立-pico-遥操) |

### PICO 与 XR Service

在 PICO 打开 XRoboToolkit，连接机器人当前的局域网 IP，开启 **Controller** 和 **Send**。计划继续做机器人遥操时，同时开启 **Head** 供 Grip 接管时标定前向。设备标识以 SDK 回调为准，下列命令使用 `TestDevice`。 [输入配置与重定位约定](SDK_INTERFACE.md#输入配置)

用 `xr_watch` 检查输入时，命令会自动准备本机服务。直接用 `xr_record` 定时录制或启动机器人遥操前，先检查服务端口。

```bash
ss -ltn 'sport = :60061'
```

端口未监听时，在容器内执行以下命令，再确认 `LISTEN`。PICO 的实际连接和输入以 SDK 回调及完整双手帧为准。

```bash
(cd /opt/apps/roboticsservice && bash ./runService.sh)
ss -ltn 'sport = :60061'
```

### 常驻输入测试

```bash
ros2 run remote_control xr_watch
```

默认使用 `TestDevice`、同步后的 `/home/agi/aimdk-fsc/src/remote_control/config/input.json` 和下节命令中的 ARM64 SDK。可用 `--device-id`、`--input-config`、`--sdk-library` 覆盖。应用重定位后，结束监听并更新 `reference_space_id` 再启动。

命令检查 `127.0.0.1:60061`，优先复用监听中的服务。需要自启时调用厂商脚本，默认等待最多 15 秒，脚本非零退出或等待超时会报错。可用 `--service-script`、`--service-start-timeout` 覆盖。退出时保留后台或已复用的服务，并清理本次启动后仍在前台运行的脚本进程组。

终端即时显示服务与设备事件，每秒显示 `window_frames`、`receive_hz`、`last_frame_age_s`、`backlog` 和最新双手柄数据。`receive_hz` 按窗口内完整帧的接收时间计算，不足两帧显示 `null`。分别移动左右手柄并操作 Grip、Trigger，确认位姿和按键更新。关闭 Send 后应看到窗口帧数归零。收到设备离线或服务断开事件时会清除显示缓存、记录失联并等待后续回调。头显重新连接并发送后检查帧是否恢复，服务进程重启后的 SDK 自动重连待现场验证。

默认持续监听，按 Ctrl-C 结束并排空原始回调，退出码为 `130`。可用 `--duration 60` 从 SDK 启动后限时，等待 PICO 发送的时间计入其中，超时仍无完整帧时返回 `1`。可用 `--display-interval` 调整显示间隔。日志默认保存为 `/home/agi/aimdk-fsc/log/YYYYMMDD_HHMMSS.jsonl`，按北京时间命名，重名追加 `_1`、`_2`。`--log-dir` 可改目录，同名 `.service.log` 用于保存自启服务的输出。

### 固定时长录制与发布

确认 PICO 已发送后，在同一终端录制 30 秒。此入口可独立使用，只需 XR Service 和 PICO 输入。`--output` 指定新文件，已有文件会被保留。

```bash
XR_LOG="/home/agi/aimdk-fsc/log/xr_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S).jsonl"
ros2 run remote_control xr_record \
  --device-id TestDevice \
  --input-config /home/agi/aimdk-fsc/src/remote_control/config/input.json \
  --sdk-library /opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so \
  --output "$XR_LOG" --duration 30
```

时长从 SDK 启动后计，窗口内无目标设备完整双手帧时返回 `1`。结束后先关闭 SDK，再排空原始回调。省略 `--duration` 可持续录到 Ctrl-C。

需要同时发布时，在上述录制命令末尾加 `--publish-xr-raw`。原始回调同时保存到 XR JSONL，并以 `std_msgs/msg/String` 发布到 `/remote_control/xr/raw`，QoS 为 Best Effort／Volatile。录 MCAP 时先对选中 `xr_raw` 的采集器发 START，等 `xr_record` 返回后发 STOP。JSONL 的 `xr_raw_publish_summary` 记录发布调用次数和首个错误，实际收到的回调数以 MCAP 读回对账为准。 [采集器启动与对账命令](../data_collection/README.md#独立-pico-输入采集)

### 日志读回与统计

对上面生成的日志执行以下命令。读回 `xr_watch` 日志时，将 `XR_LOG` 设为终端打印的实际 JSONL 路径。

```bash
ros2 run remote_control xr_stats "$XR_LOG"
```

先看 `input_frame_count`、`effective_input_hz` 和 `receive_interval_s`，再核对 `backlog`、`exit_reasons`、`session_invalidations` 和回调序号缺口。退出码 `0` 表示 JSONL 读回并完成统计。录制完整性看 `recording_complete`，Ctrl-C 收尾或记录到失联时该字段为 `false`。离线回放的 `read_frames()` 遇到失联或源时间回退会停止。 [日志字段与协议](SDK_INTERFACE.md#命令与日志)

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
