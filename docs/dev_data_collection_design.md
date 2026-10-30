# 官方原始话题采集器：需求与结构设计

状态：后续扩展设计，2026-10-30 整理。适用 SDK `aimdk v1.1.0-gfaf4c94`，执行环境为机器人 Ubuntu 22.04／ROS 2 Humble 容器。

## 1. 目标与现有实现

在保留下半身原生运控的上肢实验中，独立记录机器人命令、原始反馈和运行模式，生成可读回的 MCAP 与会话记录。采集器只读订阅官方话题，独立于其他自研包运行。

本设计面向后续研究与开发任务，定义可配置话题录制、START／STOP／STATUS、质量统计和离线读回检查。采用显式类型注册，下表六种消息类型及同类型多话题配置按后续阶段任务需求分批实现；按原消息类型保存完整字段、数组顺序和异常值。

实现位于 `overlay/src/data_collection/`，沿用 C++、rosbag2／MCAP 和 ZeroMQ。当前包提供统一 JSONL 帧及可选原始 MCAP，见[采集包说明](../overlay/src/data_collection/README.md)。本设计与该功能的集成方式列于第 8 节。

## 2. 话题与默认配置

表中消息类型使用 ROS 完整类型名。仅录制 profile 中显式启用的完整话题名。

| 官方话题 | 消息类型 | 默认配置 | 用途 |
| --- | --- | --- | --- |
| `/aima/hal/joint/arm/state` | `aimdk_msgs/msg/JointStateArray` | 启用，必需 | 双臂关节状态与错误 |
| `/aima/hal/joint/hand/state` | `aimdk_msgs/msg/HandStateArray` | 启用，必需 | 手型、夹爪／灵巧手状态、故障与触觉 |
| `/aima/mc/common/state` | `aimdk_msgs/msg/McCommonState` | 启用，必需 | 输入源、action、姿态、FSM 与运行状态 |
| `/mc/upper_body_command` | `aimdk_msgs/msg/UpperBodyCommandArray` | 启用，可选事件流 | 头部、双臂、手部目标与 source |
| `/aima/hal/joint/head/state` | `aimdk_msgs/msg/JointStateArray` | 启用，可选 | 头部反馈 |
| `/aima/hal/imu/chest/state` | `sensor_msgs/msg/Imu` | 关闭，可选 | 胸部 IMU |
| `/aima/hal/imu/torso/state` | `sensor_msgs/msg/Imu` | 关闭，可选 | torso IMU |
| `/aima/hal/joint/hand/command` | `aimdk_msgs/msg/HandCommandArray` | 关闭，可选事件流 | HAL 手部命令 |
| `/aima/hal/joint/waist/state`、`/aima/hal/joint/leg/state` | `aimdk_msgs/msg/JointStateArray` | 关闭，可选 | 腰部、腿部状态参考 |

“必需”用于判断实验数据完整性。命令—反馈对照实验将对应命令流设为必需；事件流允许长时间无消息，以会话内是否出现判断，连续缺流检查仅对配置了超时的流启用。

## 3. 配置与 QoS

profile 存放于采集包 `config/`，包含以下配置：

| 层级 | 字段 |
| --- | --- |
| 逐话题 | 稳定标识、完整话题名与类型、启用／必需标记、QoS reliability／durability／history depth、可选缺流超时、预期频率参考 |
| 全局 | 数据根目录、实验标识与说明、来源版本、队列条目与字节上限、必需状态流首次等待窗口、ZMQ 地址与端口 |

启动时校验重复条目、支持类型、目录与容量，配置冻结至节点退出；修改 profile 后重启节点。数据根目录由用户指定，持久化与同步保护沿用[采集包说明](../overlay/src/data_collection/README.md#启动与录制)。

订阅默认使用 `BEST_EFFORT + VOLATILE`，按话题允许覆盖。使用 `TRANSIENT_LOCAL` 时保留历史样本及其源时间。首版默认 `use_sim_time=false`。

SDK 本地接口文档给出的发布端配置为：关节／手部反馈与 MC 状态使用 `BEST_EFFORT + TRANSIENT_LOCAL`，上肢／HAL 手部命令使用 `BEST_EFFORT + VOLATILE`；IMU 示例使用 sensor-data QoS。部署时核对实际发布者的类型和 QoS。

类型不匹配或类型不受支持时报告配置错误。发布者数量、已确认的 QoS 不兼容和未收到消息分别记录；无法确定原因时保留为未知。多个发布者的 QoS 差异列入状态。

## 4. 内部结构

```text
profile → 配置校验／类型注册 → 多话题订阅 → 有界队列 → 单一写入线程
                                 ↑                       ↓
ZMQ START／STOP／STATUS → 会话管理器          MCAP + 接收记录 + 会话摘要
```

- 接收回调捕获时间，在会话门控内分配本地接收序号并入队；每条记录携带 session ID。
- 会话管理器串行处理 START／STOP，锁定接收边界，隔离连续会话的队列与输出。
- 写入线程独占 writer，按入队顺序写入，负责打开、排空和关闭。
- 队列同时限制条目数与字节数；满时丢弃新样本，累计逐话题与总丢弃数，继续录制。回调不等待磁盘写入。

## 5. 会话与质量状态

正常状态为 `READY → RECORDING → STOPPING → READY`，同一时刻最多一个会话。初始化、写入、关闭或不可恢复控制通道错误进入 `ERROR`，停止新样本入队，尽可能关闭文件并留存错误。修复后使用全新会话，失败会话保留。

| 操作 | 行为与响应 |
| --- | --- |
| START | 校验目录、创建唯一会话、打开 writer 并注册启用话题，随后开放接收并返回 `ACK`。ACK 表示存储就绪。录制中重复 START 返回 ACK，沿用当前会话。 |
| STOP | 关闭接收门控，排空已入队数据，关闭文件并保存摘要后返回 `ACK`；失败返回 `ERROR <reason>`。正常停止后重复 STOP 返回 ACK。 |
| STATUS | 返回 JSON：运行状态、session ID、实际路径、逐话题发布者／接收／写入／丢弃计数、最后接收时间、队列压力、缺流与错误、关闭结果。 |
| 正常退出 | 执行同一停止流程。强制终止或存储故障导致完成摘要缺失时，读回结果标为未确认完成。 |

控制客户端沿用 loopback ZMQ，默认 `tcp://127.0.0.1:5555`、响应等待 5000 ms，均可配置。STOP 超时后查询 STATUS 确认收尾状态；文件系统阻塞可能延长关闭时间。

数据质量独立于运行状态：

- 必需状态流超出首次等待窗口、或流超过已配置的缺流超时后，标记缺失并继续录制；恢复时保留历史缺失区间。
- 必需事件流在会话结束时仍未出现，标记数据不完整。启用的可选流未出现时列入摘要。
- 队列丢弃标为记录损失；源时间为零、重复或回退、序列跳变分别记录。序列跳变表示疑似缺失，本地丢弃单独计数。
- 原始异常值与设备故障字段照常保存。无数据会话允许正常关闭，摘要标为无数据。

## 6. 文件与时间语义

每次会话创建唯一目录，保存以下输出：

| 输出 | 内容 |
| --- | --- |
| `raw/` | 标准 rosbag2／MCAP，保存启用话题的原始消息 |
| `session.json` | 配置快照、环境与版本、会话摘要 |
| `events.jsonl` | 状态转换、缺流区间、录制异常 |
| `receipts.jsonl` | 每个成功写入 MCAP 样本的接收元数据，必存 |

保留消息中的 `stamp`、`meas_stamp`、`sequence`、frame 等原字段；仅检查对应类型实际具有的字段。接收时捕获 ROS、系统 UTC 与 monotonic 时间，均为整数纳秒。MCAP 存储时间取回调接收时的 ROS 时间；会话记录 `use_sim_time` 与时间基准。ROS 时间未开始、暂停或回退时，继续保存样本并标记异常。

每条接收元数据包含 session ID、话题、类型、本地接收序号、该话题写入序号和三种接收时间，以写入序号关联 MCAP 样本。附属文件写入失败按存储错误处理，读回时检查两者一致性。

会话摘要记录：

- SDK 版本、采集代码提交及 dirty 标识、机型／固件／负载，未知项记为 unknown；
- 实验说明、完整配置与 QoS、数据根目录、开始结束时间及原因；
- 各流计数与首次／末次接收、缺流与异常、队列峰值与记录损失；
- 文件关闭结果、存储错误、无数据标记及实验数据完整性。

MCAP 中的命令记录表示采集器已接收该命令。原始字段的物理解释沿用 SDK 定义。

## 7. 验收

由用户同步后在机器人 Humble 容器内验证，配套提供会话控制客户端和 MCAP 读回工具／操作说明。

| 操作 | 预期结果 |
| --- | --- |
| 独立启动，核对实际话题类型与 QoS | 接收机器人已有状态，显示发布者与接收统计 |
| START／STOP／STATUS、重复指令、连续会话、正常退出 | 响应符合第 5 节约定，会话目录唯一且无跨会话混写 |
| 对本批接入类型写入并读回，含嵌套字段和异常时间／数值 | 原始字段与数组顺序保持，MCAP 计数与接收记录、摘要可核对 |
| 以合成或隔离输入触发缺流、恢复、队列溢出、无数据及存储失败 | 质量标记、错误状态和关闭结果符合约定，历史问题保留 |
| 读回未完整关闭的会话 | 报告可读数据及未完成状态 |
| 检查实际数据目录与运行统计 | 确认持久化和同步保护，记录接收／写盘速率、数据规模与队列峰值 |

## 8. 待定项

| 决策 | 影响 |
| --- | --- |
| 原始采集替换现有统一 JSONL 采集，还是作为同包内独立运行方式 | 决定启动参数、旧功能保留与迁移方式；实施前确定 |
| QoS history depth、队列条目／字节容量、首次等待窗口与逐流缺流超时 | 依据实际频率、消息规模与观测结果确定 profile 数值；预期频率仅作参考 |

## 9. 接口依据

- 候选名称：[topics_and_services](../underlay/topics_and_services)。其中包含服务与通配项，按第 2 节配置具体话题。
- 消息定义：[JointStateArray](../underlay/src/aimdk_msgs/interface/robot/hal/msg/JointStateArray.msg)、[HandStateArray](../underlay/src/aimdk_msgs/interface/robot/hal/msg/HandStateArray.msg)、[McCommonState](../underlay/src/aimdk_msgs/interface/robot/mc/data/msg/McCommonState.msg)、[UpperBodyCommandArray](../underlay/src/aimdk_msgs/interface/robot/mc/openapi/msg/UpperBodyCommandArray.msg)、[HandCommandArray](../underlay/src/aimdk_msgs/interface/robot/hal/msg/HandCommandArray.msg)、[MessageHeader](../underlay/src/aimdk_msgs/interface/robot/common/msg/MessageHeader.msg)。
- 官方示例：[关节](../underlay/src/py_examples/py_examples/motocontrol.py)、[上肢命令](../underlay/src/py_examples/py_examples/upper_body_control.py)、[手部反馈](../underlay/src/py_examples/py_examples/omnihand_touch.py)、[手部命令](../underlay/src/py_examples/py_examples/hand_control.py)、[IMU](../underlay/src/py_examples/py_examples/echo_imu_data.py)。
- SDK 本地接口文档：`underlay/docs/cn/_build/html/zh-cn/dev/Interface/control_mod/` 下的 `joint_control.html`、`mc_status.html`、`upper_body_control.html`、`endeffector.html`。
