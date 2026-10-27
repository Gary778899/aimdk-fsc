# 项目内容
基于智元x2机器人的SDK进行一系列二次开发，包括数据采集、全身移动控制、上半身操作控制等个性化定制功能。

## 开发环境

| 环境 | 操作系统 | ROS 2 |
| --- | --- | --- |
| 本地开发电脑 | Ubuntu 24.04 | 已安装 Kilted |
| 机器人开发板 | Ubuntu 22.04 | 内置 Humble |

- 从 SDK 1.1 起，二次开发要求在机器人开发板的容器内进行。本地电脑用于编写和修改代码、脚本；项目的构建、运行和测试均在机器人开发板的容器内完成。

## 工作流程

1. 在本地工作区编写、修改代码和脚本。
2. Codex 默认只进行必要的本地静态检查；如无必要，无需执行构建、运行或测试，也无需获取容器相关信息。
3. 由用户手动使用已配置的 rsync 同步任务，将本地改动推送至机器人开发板的对应工作区，并在开发容器内构建和测试。
4. 根据用户反馈的构建和测试结果继续修改本地代码，再由用户同步并验证。用户明确要求 Codex 执行远程验证时，再确认所需环境信息并执行。

## 代码导航

以下路径均相对于工作区根目录。包职责与整理依据见 [包边界评审](docs/task_2_package_review.md)。

- `docs/`：研究规划、设计评审与实验记录。[research_plan.md](docs/research_plan.md) 是阶段目标与状态入口；[task_2_package_review.md](docs/task_2_package_review.md) 记录当前包边界、已知缺口及整理建议。`docs/cn/_build/html/` 为随 SDK 提供的本地接口文档；查阅时核对 SDK、固件与机型适用范围。
- `docs/archive/`：历史设计、接口核实与实验记录。可用于追溯依据，但旧包名、命令、迁移方案和验证结果不直接作为当前用法或验收结论；当前入口优先查看对应包 README，并核对源码。
- `extra/`：独立的强化学习参考工程；`mc-rl/` 包含训练、仿真验证和部署参考，`x2_rl_deploy/` 提供 ONNX 策略部署与 MuJoCo 示例。目录含 `COLCON_IGNORE`，其中还有另一份 `aimdk_msgs`；不属于当前 `src/` 包集合，不将其依赖、模型或真机结论直接套用到当前工程。
- `src/`：当前 ROS 2 工作区的源码包，保持平铺结构；下表按来源和职责分类，不代表需要移动目录。

| 分类 | 源码目录 → ROS 2 包名 | 职责与修改入口 |
| --- | --- | --- |
| 官方 SDK | `src/aimdk_msgs/` → `aimdk_msgs` | 厂商消息、服务与接口生成／预编译支持。接口定义在 `interface/`，构建逻辑在 `CMakeLists.txt` 和 `cmake/`；核对协议从这里开始。 |
| 第三方依赖 | `src/ruckig/` → `ruckig` | SDK 携带的在线轨迹生成库；业务代码按需依赖，不在其中放实验逻辑。 |
| 官方示例 | `src/examples/` → `examples`；`src/py_examples/` → `py_examples` | C++／Python SDK 调用示例，用于理解接口；自研功能分别进入下述自研包，不把示例模块作为公共库。 |
| 官方诊断工具 | `src/robotop/` → `robotop` | 官方 SDK 自带的终端状态、日志、通信与故障诊断工具，以及独立 Orin 资源监视节点；不承载自研采集或实验状态机。 |
| 自研实验工具 | `src/test/` → `aimdk_test` | 头部、手臂诊断与运动实验。入口为 `aimdk_test/head_test.py`、`arm_test.py`；`*_core.py` 为无 ROS 导入的校验／算法核心，`arm_test_motion.py` 为手臂 ROS 执行适配，`head_test_diagnostics.py` 提供实验 JSONL 日志。这里的 `test` 是业务包目录，不是全工作区单元测试目录。 |
| 自研功能命令 | `src/functions/` → `aimdk_functions` | 独立实用功能：`volume_control.py`（音量／静音）、`dual_arm_control.py`（URS 下单次关节增量／夹爪开度）。双臂纯计算与反馈校验在 `dual_arm_control_core.py`，日志在 `dual_arm_control_diagnostics.py`；不依赖测试包，也不作为所有公共代码的汇总包。 |
| 自研只读观测 | `src/locomotion_wrapper/` → `locomotion_wrapper` | 厂商关节状态聚合与标准 IMU 校验转发，不含运动控制或策略推理。节点在 `src/hardware_wrapper_node.cpp`，纯校验在 `include/locomotion_wrapper/state_validation.hpp`，配置在 `config/`，启动在 `launch/`，校验测试在 `test/`；`joint_watch.py` 仅监视标准关节状态。 |
| 自研数据采集 | `src/data_collection/` → `data_collection` | 独立订阅 SDK 双臂／夹爪反馈，生成统一 JSONL 帧并可选录制原始 MCAP，通过 ZeroMQ START／STOP／STATUS 控制录制。节点在 `src/data_collection_node.cpp`；`launch/data_collection.launch.py` 单独启动采集器，`robot_data_collection.launch.py` 为同包内的启动别名。 |

新增 `aimdk_test`／`aimdk_functions` 命令时，将模块放在相应 Python 包中，在各自 `setup.py` 的 `console_scripts` 注册，并在 `package.xml` 声明依赖；命令使用 ROS 2 包名，而非外层目录名，如 `ros2 run aimdk_test arm_test`。

`aimdk_bringup`、`aimdk_common`、`aimdk_test/common/` 和根目录 `tools/` 当前尚不存在，文档中的相关内容属于后续建议。采集与观测 README 引用的 `docs/humble_baseline.md`、离线控制／导出工具及固定 `install-humble` 布局不能当作当前已具备的能力；使用前核查文件和实际配置。

## 关键约束

以下约束依据当前文档与源码，描述既有接口和行为；不代表已完成相应真机验收。修改相关代码时应保持这些约束；若任务明确要求改变契约，同步更新相关实现、验证与文档。

### 包边界与构建范围

- 自研业务不依赖官方示例作为公共库，不向 `aimdk_msgs`、`ruckig` 或 `robotop` 混入实验业务。公共逻辑先在实际使用的包内整理；不要仅为目录分类新建包或搬动源码。`head_test_core.py`、`arm_test_core.py`、`dual_arm_control_core.py` 保持无 ROS 导入，ROS 服务、发布和执行状态机留在适配层。依据：[包边界与公共能力评审](docs/task_2_package_review.md)、[功能包说明](src/functions/README.md)。
- 容器构建命令显式限定 `--base-paths src`，避免纳入 `extra/` 独立工程及同名包。目录改名或依赖整理需同时核查入口、导入、日志路径探测和安装来源；受同步排除保护的旧安装产物不会自动清理。依据：[同步与构建要求](docs/task_2_package_review.md#6-同步构建与版本追溯)。

### 上肢实验与反馈

- `aimdk_functions dual_arm_control` 是单次指令入口，支持七关节 tag、左／右／双侧及夹爪开度。要求操作者预先进入 URS/RUNNING，默认只读，显式 `--execute` 才发布；关节相对增量绝对值不超过 10°，采用五次插值且不自动返程或切换模式。限位按 SDK 1.7.1 与官方示例的交集及左右侧映射检查选定关节的起点、目标；未选关节和夹爪保持采样目标、不裁剪。旧双臂顺序、肩方向及独立夹爪入口已移除；`arm_test` 专项接管／交接诊断保留。新入口与旧流程的验证结论分开记录。依据：[功能包说明](src/functions/README.md)。
- `head_test`、`arm_test` 默认只读，运动保留显式 `--execute` 开关，真机运动由用户明确启动。当前上肢实验保留下半身原生运控，通过 `/mc/upper_body_command` 控制，不改为 HAL 关节命令、不停止 MC。模式切换须有界查询实际目标模式及 `RUNNING`，不能仅凭 Set 服务成功回包认定接管完成。停止发布或请求 `STAND_DEFAULT` 都不能当作已验证的急停／必达恢复。依据：[研究规划](docs/research_plan.md)、[上肢接口核实](docs/archive/upper_body_interface_findings.md)。
- 上肢位置命令使用 rad，`head_pos` 为两槽，`arm_pos` 为左臂 7 关节后右臂 7 关节；反馈按关节名称匹配并重排，不能依赖输入数组顺序。单臂实验仍构造完整双臂目标；夹爪模式 1 的 `hand_pos` 为左右开度，范围 0..1，不能将 0 模式或空数组解释为通用保持。重发当前开度只是位置目标，不等同于夹持力保持；固定消息槽位也不代表当前机型开放所有关节。依据：[上肢接口核实](docs/archive/upper_body_interface_findings.md)、`UpperBodyCommandArray.msg`、`arm_test_core.py`。
- 控制反馈保留源时间，分别检查接收时效、源时间年龄及时间推进；不能用本机当前时间覆盖源时间来掩盖断流、冻结或回退。保留测试包的错误锁存及异常中止行为，不以观测包可自动恢复的无状态年龄校验替代。FSM 的源码与历史文档编码有冲突，未核实前不得把 raw=4 判作稳定，也不擅自恢复现有脚本已移除的 FSM 门槛。依据：[接口核实](docs/archive/upper_body_interface_findings.md)、[观测限制与公共能力评审](docs/task_2_package_review.md)。
- 当前上肢消息提供位置目标，没有力矩前馈或增益字段；不能凭 HAL 消息存在相应字段就认定原生上肢链路支持。用于力估测前须核实 `effort` 的物理含义、单位、符号及传动侧定义；位置偏置修正不表述为已实现通用重力力矩补偿。依据：[研究规划任务 6](docs/research_plan.md#任务-6模型辨识重力补偿与末端三维力估测)。

### 观测与采集语义

- 观测包保持只读；采集器不触发运动、不切换模式，独立订阅 SDK 的 `JointStateArray`／`HandStateArray`，由 JSON 配置映射双臂与夹爪字段，不依赖观测包。改变话题或消息类型字符串不能自动支持其他消息结构；`robot_data_collection.launch.py` 当前仅为独立采集入口别名。包名已由 `data_collection_edge_agent` 改为 `data_collection`，同步后需核查旧安装产物与 overlay 来源，再在容器构建验证。依据：[采集说明与迁移步骤](src/data_collection/README.md)。
- `config/wbc_observer.yaml` 当前为腿部／胸部 IMU profile；上肢采集须配置真实输入话题和核实过的完整关节名称集合，不能只打开 `require_arm`。`expected_names` 当前约束集合而不保证输出顺序，消费者按 `name` 映射；胸部与 torso IMU 不默认等价。聚合使用最早源时间，MCAP 存储时间为节点接收时间，两者不等同于硬件同步；标准消息也不能替代原始厂商错误、夹爪、模式和命令记录。依据：[观测与数据边界评审](docs/task_2_package_review.md)。
- ZeroMQ `START` 的 `ACK` 只表示 writer 初始化成功，不证明已收到有效样本；录制成功需结合实际数据及读回验证。录包目录显式选择已确认的持久化位置；当前工作区内 `rosbags/` 未受同步排除保护，不能直接假定安全。若要使用该目录，先为预览和推送同时配置保护性排除；保留现有同步规则，除非任务明确要求调整。依据：[采集说明](src/data_collection/README.md)、[数据与同步评审](docs/task_2_package_review.md#6-同步构建与版本追溯)、`.vscode/tasks.json`。


## 同步配置

同步任务已配置在 `.vscode/tasks.json` 中：

- `机器人：预览同步`：使用 rsync 的 `--dry-run` 预览待同步内容。
- `机器人：推送工作区`：实际推送当前工作区。

当前配置将工作区同步至 `agi@10.0.1.41:/home/agi/aimdk-fsc/`。预览与推送均使用 `--delete`，清理目标目录内本地已不存在且未被排除规则保护的内容；现有排除项保持不变，不使用 `--delete-excluded`。连接地址、目标路径及排除规则以 `.vscode/tasks.json` 的实际配置为准；执行同步时复用现有配置。 

机器人宿主机的同步目录与容器内的项目路径相同。仅在任务确实需要 Codex 执行容器内操作时，才确认实际容器挂载配置、容器名称、进入方式和容器内路径，不要自行假定；仅编写脚本和进行静态检查时，无需获取这些信息。

## Codex 执行要求

- 编写或修改脚本时，文件内的所有函数（包括类方法、构造函数和嵌套函数）都必须提供 docstring，说明其用途；有参数、返回值或重要异常时，也应说明其含义。Python docstring 应放在函数体的第一条语句位置，不能仅用普通注释代替。使用中文编写注释说明。
- 涉及 ROS 2、SDK 或机器人依赖的代码，应面向机器人开发板容器内的实际环境编写；不要仅凭本地安装了 Kilted 就引入 Kilted 专属接口或依赖。
- 编写或修改脚本后，默认只进行必要的文件检查、语法检查、代码审阅等静态检查；项目构建、运行和测试默认由用户手动同步后在机器人开发板容器内完成。
- 不要将本地检查结果表述为机器人容器内构建或测试通过。汇报验证情况时，明确执行环境、已完成的检查和未执行的验证。
- 仅在任务确实需要远程连接、容器或挂载信息时，先检查项目现有配置和文档；仍无法确认时再向用户询问。不要因缺少这些信息而阻塞不依赖它们的代码编写和静态检查，也不要猜测执行命令或声称已完成验证。
- 开发优先实现最小功能闭环。新增安全机制、保护检查或中止门槛前，先参考相近官方示例，结合接口要求、当前实验目标和已观察到的问题评估必要性。缺乏明确依据的稳定性、时序或精度阈值，优先记录观测结果，不默认作为阻断条件；根据实测需要逐步增加，并说明依据。已有明确约定的接口约束和异常处理，不因本原则自动移除。
