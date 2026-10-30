# 官方原始话题采集器：需求与结构设计

设计日期：2026-10-23。适用当前官方 SDK `aimdk v1.1.0-gfaf4c94`，目标执行环境为机器人 Ubuntu 22.04／ROS 2 Humble 容器。本文是下一步实现依据，所有新增行为均为设计要求，不表示已经编程或验收。

## 1. 目标与边界

在保留下半身原生运控的上肢实验中，独立记录“实际发送了什么、机器人反馈了什么、当时处于什么模式”，形成可读回的 MCAP 和可追溯的会话记录。采集器直接订阅官方话题，不依赖 `locomotion_wrapper`、`aimdk_test`、`aimdk_functions`、`robotop` 或 ONNX 模型。

首版范围：原始话题录制、显式 START／STOP、录制状态查询、数据完整性统计、异常留证和最小离线读回检查。采集器不会发送机器人命令、调用运动服务、切换模式或启动控制程序；对命令话题的订阅只是旁路记录。

首版不包括自动录制所有话题、相机／点云／音频、标准状态转换、轨迹生成、控制精度判定、在线同步融合、CSV 分析平台、图形界面、服务调用追踪和动作回放。后续扩展不改变独立采集的包边界。

## 2. 话题清单与接口依据

[topics_and_services](../underlay/topics_and_services) 提供候选名称，但混合了话题与服务、也包含通配路径，不能直接作为完整且已验证的录制配置。下表结合当前 SDK 消息定义、官方示例和随 SDK 提供的本地接口文档核对；“源码确认”不等于实际机器人已发布，部署时仍核对实际类型和 QoS。

| 官方话题 | 类型 | 首版配置建议 | 用途与依据 |
| --- | --- | --- | --- |
| `/aima/hal/joint/arm/state` | `aimdk_msgs/msg/JointStateArray` | 默认启用，必需流 | 双臂名称、位置、速度、effort、错误和消息头；[关节示例](../underlay/src/py_examples/py_examples/motocontrol.py) 列出关节组输入，消息见 [JointStateArray](../underlay/src/aimdk_msgs/interface/robot/hal/msg/JointStateArray.msg) |
| `/aima/hal/joint/hand/state` | `aimdk_msgs/msg/HandStateArray` | 默认启用，必需流 | 左右手型、夹爪／灵巧手数组、故障及触觉字段；[官方手部示例](../underlay/src/py_examples/py_examples/omnihand_touch.py)、[消息定义](../underlay/src/aimdk_msgs/interface/robot/hal/msg/HandStateArray.msg) |
| `/aima/mc/common/state` | `aimdk_msgs/msg/McCommonState` | 默认启用，必需流 | 输入源、action、姿态、FSM 和运行状态；候选清单、本地 `mc_status.html` 及 [消息定义](../underlay/src/aimdk_msgs/interface/robot/mc/data/msg/McCommonState.msg) |
| `/mc/upper_body_command` | `aimdk_msgs/msg/UpperBodyCommandArray` | 默认启用，按事件记录，可在命令跟踪实验中设为必需 | 实际发布的头、双臂、手部目标及 source；[官方上肢示例](../underlay/src/py_examples/py_examples/upper_body_control.py)、[消息定义](../underlay/src/aimdk_msgs/interface/robot/mc/openapi/msg/UpperBodyCommandArray.msg) |
| `/aima/hal/joint/head/state` | `aimdk_msgs/msg/JointStateArray` | 默认启用，可选流 | 头部实验反馈；同一官方关节接口，不能假定所有机型均有两个有效头部关节 |
| `/aima/hal/imu/chest/state` | `sensor_msgs/msg/Imu` | 默认关闭，按实验显式启用 | 原始胸部 IMU；[官方 IMU 示例](../underlay/src/py_examples/py_examples/echo_imu_data.py) |
| `/aima/hal/imu/torso/state` | `sensor_msgs/msg/Imu` | 默认关闭，按实验显式启用 | 原始 torso IMU；与胸部 IMU 不视为等价，不猜测外参 |
| `/aima/hal/joint/hand/command` | `aimdk_msgs/msg/HandCommandArray` | 默认关闭，按需启用，可选事件流 | 记录实际存在的 HAL 手部命令；[官方手部示例](../underlay/src/py_examples/py_examples/hand_control.py)、[消息定义](../underlay/src/aimdk_msgs/interface/robot/hal/msg/HandCommandArray.msg)；启用记录不触发任何控制 |
| `/aima/hal/joint/waist/state`、`/aima/hal/joint/leg/state` | `aimdk_msgs/msg/JointStateArray` | 默认关闭，必要时作为姿态／扰动参考启用 | 官方关节接口扩展，不开展下半身策略控制，也不作为上肢采集启动前提 |

“必需流”用于判断会话数据是否满足所选实验 profile，不意味着缺失时停止所有记录。命令话题在实验尚未执行时可以没有发布者，默认不据此认定采集器故障；需要命令－反馈对照的实验应将其声明为必需，并在会话结果中明确是否实际收到命令。

首版选择**显式类型注册与可配置话题清单**：支持上表所需的六种类型，不只修改旧参数中的话题名称；同类型可配置多个话题。保留消息全部原始字段，不重排关节、不补零、不归一化、不将厂商消息转成标准 `JointState`。暂不为任意未知类型建立自动发现录制框架。

`/aima/hal/joint/*/command` 是通配名称，不自动展开或默认全录；其他 HAL 关节命令、图像、雷达、HDS 等按后续实验需要单独增加接口与 profile。首版不要求支持它们。

## 3. 配置与 QoS

配置 profile 属于采集包自己的 `config/`，与观测包 profile 无关。每个条目至少声明：稳定标识、完整话题名、完整消息类型、是否启用、是否为实验必需流、QoS reliability／durability／history depth、可选缺流超时与预期频率参考。

全局配置包括：数据根目录、实验标识／说明、启动来源版本、队列条目／字节容量、写入失败策略、必需流首次等待窗口、ZMQ 地址／端口。配置在启动时校验，会话期间冻结；更换 profile 通过停止后重新启动节点完成，首版不提供录制中热切换。

当前 SDK 本地接口文档中：关节反馈、手部反馈、MC 状态标为 `BEST_EFFORT + TRANSIENT_LOCAL`；上肢与 HAL 手部命令标为 `BEST_EFFORT + VOLATILE`。官方 IMU 示例使用 sensor-data QoS。它们是 profile 初值依据，实际发布者仍需在容器中核对，不能把自研测试订阅参数当作官方唯一依据。

建议首版默认订阅 reliability 使用 best-effort；默认 durability 使用 volatile，以接收订阅建立后可获得的数据，避免把历史缓存当作会话内新反馈。确需获取 transient-local 缓存时按话题显式开启，并记录 QoS 与首次样本源时间。新建会话不承诺发布者缓存已清空，也不根据源时间自动丢弃较旧样本；离线可辨别源时间早于会话开始的样本。

发现已存在发布者的类型与配置不一致或当前无法支持该类型时，报告配置／类型错误，不强行录错类型。没有发布者、QoS 不兼容、未收到数据分别报告；“没有消息”本身不能证明具体原因。多个发布者使用不一致 QoS 时明确报告，首版不承诺对所有组合自动适配。

## 4. 功能需求

| 编号 | 首版必须满足 |
| --- | --- |
| R1 独立启动 | 只加载采集器自身配置及官方接口，不启动观测、控制或模型节点；不因其他自研包未安装而无法使用 |
| R2 原始留存 | 完整记录配置内已接收的原始消息，包括错误状态、异常数值、重复／零／回退时间；不因数据无效过滤消息 |
| R3 显式会话 | 每次有效 START 创建唯一会话；不覆盖既有数据；同一时刻最多一个录制会话 |
| R4 可观测状态 | 状态查询包含录制状态、session ID、实际路径、逐话题发布者／接收／写入／丢弃计数、最后接收时间、必需流缺失和错误原因 |
| R5 有界资源 | 接收与磁盘写入分开；设置队列条目和字节上限，不能无限占内存；记录过载造成的损失 |
| R6 可靠停止 | STOP 明确定义停止接收边界、排空队列、关闭 writer 和保存摘要；ACK 表示已完成关闭，失败返回错误 |
| R7 元数据 | 每次会话保存完整配置、版本信息、QoS、时间语义、开始／结束原因、计数、质量状态及可读回线索 |
| R8 旁路完整性 | 数据到达、缺流、队列溢出和源序列／时间异常仅作标记，不向机器人发送恢复或重试控制命令 |
| R9 最小操作工具 | 配套提供 START／STOP／STATUS 客户端及 MCAP 读回检查说明；只靠仓库中实际存在的工具即可完成最小闭环 |

首版新增的字段检查仅覆盖消息头时间／序列与录制运行状态，不复刻前期测试脚本的关节安全范围、夹爪静止、模式接管和轨迹判定。NaN、故障码或状态转换是要记录的证据，不等于采集器故障。

## 5. 结构与模块职责

以下为采集包内部模块，不是新增 ROS 包：

```text
采集 profile ──→ 配置校验／类型注册 ──→ 多话题订阅
                                         │
                              接收计数、时间信息、会话编号
                                         │
                                      有界队列
                                         │
                                  单一 MCAP 写入线程
                                         │
                              录包 + 会话元数据 + 事件记录

ZMQ START／STOP／STATUS ──→ 会话管理器 ──→ 接收门控／写入器／状态快照
```

| 模块 | 职责与边界 |
| --- | --- |
| 配置与接口注册 | 管理官方六种类型、话题及 QoS；检查重复条目、未知类型、无效目录／容量配置，记录实际发布者信息 |
| 接收层 | 捕获消息及接收时间；在会话门控内编号并入队，不同步等待磁盘；不修改原始业务字段 |
| 会话管理器 | 串行化 START／STOP，分配 session ID、定义接收边界、防止上一会话队列写入下一会话；不在控制 socket 处理逻辑内直接操纵机器人 |
| 写入层 | 单线程拥有 writer，按入队顺序写入，统一处理 open／write／close 失败及排空；不能由多个回调竞争 writer 生命周期 |
| 状态与质量统计 | 记录流是否出现、当前缺流、累计间隔、队列压力、丢弃及写入错误；即使原始消息异常也保持录制 |
| 会话存储 | 保存配置快照、事件与最终摘要，关联 MCAP；仅负责数据管理，不承担曲线分析或硬件同步 |
| 控制客户端 | 仅控制本采集器会话；保留现有 loopback ZMQ 方式，首版不额外引入 ROS 自定义控制服务 |

## 6. 会话生命周期与完整性

录制运行状态：`READY → RECORDING → STOPPING → READY`；初始化、写盘、存储或不可恢复控制通道错误进入 `ERROR`。恢复时创建全新会话，保留失败会话，不原地覆盖；文件系统状态不明确时不自动重试录制。

数据质量与运行状态分开：必需流首次等待、缺流、记录损失、源时间异常等独立列出，历史问题在最终摘要保留。窗口内未收到必需流时继续保存其他流，并标记该实验数据不完整；后续恢复不抹除先前缺失记录。可选流未出现不判整个会话失败；启用但缺失仍列在摘要中。

- **START**：校验配置与目录、打开 writer、注册所有启用话题、建立 session ID 后开放接收。保持已有 `ACK`／`ERROR <reason>` 响应形式；ACK 仅表示录制已就绪，不表示必需流齐全或机器人正在运动。重复 START 不创建新会话。
- **STOP**：先关闭当前会话接收门控；已入队消息继续排空，成功关闭 MCAP 并保存摘要后返回 ACK。重复 STOP 在正常已停止状态下幂等。排空／关闭失败返回 ERROR，保留错误记录；不能声称完整收尾。
- **STATUS**：新增独立查询，返回结构化状态，不改变 START／STOP 的现有响应。客户端具有有限响应等待时间；STOP 等待上限和超时后的状态查询流程需明确，客户端超时不能被解释为 writer 已关闭。
- **退出**：正常信号退出执行同一停止流程。强制终止、断电、磁盘不可写可能无法生成最终摘要；“缺少完成摘要”本身是未确认完成的证据，不能补写为正常完成。

有界队列默认过载策略建议为丢弃新入队样本、累计逐话题和总损失计数、将会话标为存在记录损失，继续保留其他证据；不阻塞订阅回调等待磁盘。STOP 后停止新样本入队，队列本身有界。底层文件系统故障仍可能阻塞关闭，不能承诺硬实时 STOP。

每个入队记录携带会话编号与本地接收序号，STOP／START 并发边界由管理器锁定。接收顺序不等于各话题源时间顺序；首版不做跨话题重新排序。

## 7. 数据、时间与落盘契约

每个会话目录建议包含：标准 rosbag2／MCAP 输出、`session.json` 配置及会话摘要、`events.jsonl` 运行事件，以及在需要保存逐样本接收信息时的 `receipts.jsonl`。具体 bag 子目录名在实现时统一，不要求创建多个互相依赖的业务包。

- 原始消息头里的 `stamp`、`meas_stamp`、`sequence` 和 frame 不改写；标准 IMU 没有厂商 `MessageHeader` 的扩展字段，不为其伪造字段。
- 接收时分别捕获 ROS 时间、系统 UTC 时间与 monotonic 时间，均用整数纳秒。MCAP／rosbag 存储时间使用回调接收时捕获的 ROS 时间，而不是磁盘线程实际落盘时间；另外记录 `use_sim_time` 和时间基准。
- 每个已成功写入样本的接收元数据关联话题、本地接收序号和该话题写入序号，便于区分重复源时间；记录类型、会话号及失败情况，不能仅以源时间作唯一键。附属文件写入失败同样使会话无法标为完整，避免 MCAP 与摘要悄然不一致。
- 首版 profile 面向真机、默认不使用仿真时间；若 ROS 时间未开始、暂停或回退，保留可记录样本并显式标记，不能据此声称设备源时间与本机时间同步。回放仿真时间的专项行为后续单独验证。
- 序列跳变只是“疑似缺失”线索：发布者重启、多发布者、回绕和定义差异均可能导致跳变。接收计数、写入计数和本地丢弃可计量，但无法据此保证网络端到端无丢包。
- MCAP 保存命令仅证明该消息被采集器收到，不证明原生控制器接受或执行；`effort` 仍按原字段保存，不擅自认定是真实力矩或接触力。

会话元数据至少记录：SDK 版本、采集代码提交及 dirty 标识、用户填写的固件／机型／负载信息（未知明确记为未知）、实验说明、配置与 QoS、数据根目录、各流计数／首次及末次接收、开始结束时间、停止原因、异常／缺流区间、是否成功关闭和是否存在记录损失。

没有任何样本的会话可正常关闭，但摘要必须标为无数据；“MCAP 文件存在”不作为实验数据采集通过条件。

## 8. 独立部署与依赖调整范围

实现时保留 C++ 采集节点，基于 `rclcpp`、官方 `aimdk_msgs`、`sensor_msgs`、rosbag2／MCAP 及 ZeroMQ 构建，核对并补齐 `cppzmq-dev` 等开发依赖；不依赖其他自研包。消息类型支持、MCAP 元数据及附属文件一致性以 Humble 容器中的实际库行为验证。

采集包拥有自己的 launch、profile 和最小操作说明。旧 `robot_data_collection.launch.py` 不再自动拉起观测包：实现时可作为独立采集入口的兼容别名，或明确停用并提供迁移说明。现有两路标准消息参数不再作为新上肢 profile 的必需接口；旧配置如保留，需标明类型及用途，不能静默改变其含义。

数据根目录必须由用户指定到已确认持久化的位置，不能默认为工作区内不受保护的 `./rosbags`。本设计不预设宿主机和容器挂载路径。`.gitignore` 不替代 rsync 数据保护规则。

## 9. 验收范围与实施前核对

后续由用户同步并在机器人 Humble 容器内验证，本轮不执行：

1. 不启动观测包或测试脚本，仅运行采集器也能接收机器人已有原始状态；缺少命令发布者时正确报告可选流等待。
2. 核对上表实际话题类型、发布者 QoS 和可获得频率，记录机型与固件差异；本地文档中的典型频率不作为已实测指标。
3. START／STOP／STATUS、重复指令、连续多会话和正常退出结果符合约定，无跨会话混写。
4. 六种类型均可序列化／反序列化并读回关键与嵌套字段；真实存在的非有限值、故障码、重复／零时间不会被静默清洗。
5. 用合成／隔离数据验证必需流缺失和恢复、队列溢出、打开／写入／关闭失败、附属文件失败、无数据和未完整关闭的识别，不发送运动指令来验证采集器。
6. MCAP 逐话题计数与成功写入记录、接收／丢弃摘要可核对；能定位命令与反馈及模式时间线，之后再开展完整分析工具。
7. 确认所选数据目录具备实际持久化与同步保护；记录数据规模、接收／写盘速率和队列峰值后，再确定可声明的吞吐与存储能力。

当前用户已确定“直接原始采集、后置模型链路、不重构验证脚本”。表中的首版启用集合、旁路统计、队列与状态接口是本设计建议；实际实现前还需确定持久化数据路径、实验是否要求命令／IMU必需，以及机型、固件对应的在线接口信息。它们不阻塞本轮文档设计，也不以猜测的远程环境代替确认。

## 10. 接口依据

- [官方关节示例](../underlay/src/py_examples/py_examples/motocontrol.py)、[官方上肢示例](../underlay/src/py_examples/py_examples/upper_body_control.py)、[官方手部反馈示例](../underlay/src/py_examples/py_examples/omnihand_touch.py)、[官方 IMU 示例](../underlay/src/py_examples/py_examples/echo_imu_data.py)。
- [MessageHeader](../underlay/src/aimdk_msgs/interface/robot/common/msg/MessageHeader.msg)、[JointState](../underlay/src/aimdk_msgs/interface/robot/hal/msg/JointState.msg)、[HandState](../underlay/src/aimdk_msgs/interface/robot/hal/msg/HandState.msg) 及上文链接的数组与 MC 接口。
- SDK 随附本地接口文档：`underlay/docs/cn/_build/html/zh-cn/dev/Interface/control_mod/` 中 `joint_control.html`、`mc_status.html`、`upper_body_control.html`、`endeffector.html`。此目录已被 Git 忽略，需从对应官方 SDK 资源恢复；文档约定仍须以在线环境核对。
