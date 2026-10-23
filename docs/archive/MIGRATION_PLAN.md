# WBC middleware 部分迁移计划

源项目：`/home/gary/projects/WBC_middleware`，SDK `v0.9.0-g765f14d`，提交 `a8b4b71`。
目标项目：`/home/gary/projects/aimdk-sdk-v1.1`，SDK `v1.1.0-gfaf4c94`。

## 目标与边界

将旧项目的自定义状态观测、采集、离线工具和只读策略推理迁入当前 SDK。旧项目保持不变；目标项目沿用现有 `aimdk_msgs`、`ruckig`、示例和虚拟环境。执行迁移不连接机器人，不发布关节命令，不切换运动模式。

迁移后默认运行链路是：厂商状态 → 带时效校验的标准状态 → MCAP 采集 / 离线导出；另提供厂商腿部和 torso IMU 状态 → 策略影子推理。策略输出仅用于检查，不执行控制。

## 已验证的差异与问题

- 旧消息定义 56 个，新定义 121 个；共同定义中 45 个去除注释后相同，8 个发生变化，旧定义有 3 个被移除。
- `JointState` 删除 `coil_temp/motor_temp/motor_vol`，新增 `error_code`。`JointStateArray` 文本虽相同，其嵌套类型已变化，所有迁移代码必须用新 SDK 编译，不能叠加旧消息包。
- `SetMcPresetMotion` 增加输入源和播放时间字段；旧 `PlayMediaFile`、`LedStripCommand` 已不在新接口中。旧 SDK 示例不整包覆盖新示例。
- 两项目的 Ruckig 源文件一致，无需迁入第二份。
- `locomotion_wrapper` 在临时构建目录、链接新 SDK 且 `BUILD_TESTING=OFF` 时编译成功；打开测试时因引用已被旧提交删除的 `hardware_wrapper_node_mock_test.cpp` 而失败。
- 旧 wrapper 以 2ms 定时器重发缓存、覆盖源时间戳、允许不完整关节组、没有超时门限；监视工具默认话题与 wrapper 不一致。胸部 IMU 与策略使用的 torso IMU 不应混淆。
- 采集节点当前配置因缺少 `libzmq` pkg-config / 头文件失败；Humble 的 `TopicMetadata` 聚合初始化不适用于本机 Kilted 的结构布局。其线程 detach、通用 ACK、秒级文件名和默认 `0.0.0.0` 控制监听还需修正。
- `wbc_middleware` 尚非 ROS 包；策略脚本没有命令发布器，但传感器 ready 检查被注释，模型路径依赖 cwd，关节完整性 / 新鲜度未验证；`test.py` 在初始化 LSTM 状态前调用重置方法。
- 离线导出丢失关节名字，按最近邻时间合并可能复用样本，类型库固定为 Humble。应保留语义、显式标注对齐误差，不能把对齐视为硬件同步。
- 初始目标虚拟环境缺少 `onnxruntime`、`pyzmq`、`pandas`、`rosbags`；按模块需要增加依赖，不能因此替换系统 ROS 2 包。

## 执行分工（先计划，再启动子代理）

### A：状态观测包

只负责 `src/locomotion_wrapper/`。迁入只读节点和监视工具，明确 C++17。保留源时间戳，IMU 输入可配置；校验时间戳、完整性、重复名字、数值有效性、域 / 关节错误和缓存时效。默认失效时停止有效状态输出并报告原因，不能将断流数据刷新成新样本。头、臂、腰、腿是否必需及容许时间差可配置。修复监视话题。提供实际存在的测试，覆盖陈旧数据、缺组和映射。不要依赖真实 DDS 网络来完成核心算法单元测试。

### B：采集与工具

只负责 `src/data_collection_edge_agent/`、`tools/edge_agent_control.py`、`tools/mcap_to_robot_csv.py`，可添加自身测试与说明。兼容当前 rosbag2 API，显式设置 metadata 字段；使用可 join 的线程、异常后明确错误响应、唯一 bag 名称、可靠关闭，控制端默认 loopback 并允许配置。保留 ZMQ 协议，若开发依赖无法取得，必须明确报告，不静默换协议。离线导出保留 joint name、position、velocity、effort、原始时间及对齐偏差；处理缺失、空数据和重名等情况。launch 的输入、输出话题保持一致，记录依赖及安装方法，不自动安装系统包。

### C：只读策略包

只负责 `src/wbc_middleware/`。把 `control_middleware.py`、`policy.onnx`、配置封装为独立 ROS Python 包（可保留兼容入口）。不迁入旧 `test.py`，不包含任何动作发布器或运动服务客户端。参数化模型路径和配置，以安装资源定位；检查模型真实输入输出、47 维观测、12 维动作、LSTM 状态形状 / dtype。完整、有效、新鲜且时间相容的 torso IMU / 12 关节状态齐备后才推理，断流 / 重启后重置循环状态。训练频率、关节顺序、重力和相位约定不能凭猜测改变，未确认项明确记录。添加独立于 ROS 网络的观测和状态管理测试。

## 主代理验收

1. 审阅三个子代理的修改范围、实际源码与风险，确认没有动作下发接口，没有旧消息 / SDK 副本。
2. 检查目标构建环境和 Python 依赖；允许在项目虚拟环境安装必要 Python 依赖。缺少系统依赖时优先使用可用的本地开发文件；不得修改系统或伪造依赖。
3. 用新 `aimdk_msgs` 构建所有迁入包，默认测试配置可生成；执行有意义的纯逻辑测试、Python 入口导入 / CLI 帮助、模型推理（运行时可用时）及 MCAP 往返（依赖可用时）。
4. 确认旧项目 `git status` 不变，新 SDK 原有接口检查继续通过。
5. 编写迁移结果、运行命令、已测 / 未测项目和剩余依赖。没有真机与训练基准时，不将编译 / 影子推理通过表述为真机控制可用。

## 后续阶段

先在隔离 ROS domain 或仿真 / 录包回放中验收数据质量，再连机器人做纯订阅检查。真实反馈率、丢包、延迟、时间源、IMU 外参和机型关节配置必须实测。最后才单独设计控制出口、限幅、失效策略、控制权切换和停止方案；这部分不属于本轮迁移。

旧 `mc_action_proxy.py` 涉及运动模式服务转发，旧 `play_media` 依赖移除接口，不纳入默认运行链路。`extra/x2_rl_deploy` 可参考，但其说明明确示例只在仿真验证，不能直接视为真机控制方案。

## 执行状态

- 计划生成后已启动三个子代理，分别交付 A / B / C；主代理提出完整性、时钟与安装入口方面的验收反馈并复验。
- A / C 已完成构建安装与逻辑测试验收；B 工具验证通过，采集 C++ 构建等待系统 ZeroMQ 开发文件，不计作通过。
- 详细结果及剩余事项见 `MIGRATION_REPORT.md`。
- 临时编译证据：`/tmp/wbc-migration-build.log`、`/tmp/wbc-migration-testing.log`。
