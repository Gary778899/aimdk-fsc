# 项目内容
基于智元x2机器人的SDK进行一系列二次开发，包括数据采集、全身移动控制、上半身操作控制等个性化定制功能。

## 开发环境

| 环境 | 操作系统 | ROS 2 |
| --- | --- | --- |
| 本地开发电脑 | Ubuntu 24.04 | 已安装 Kilted |
| 机器人开发板 | Ubuntu 22.04 | 内置 Humble |

- 从 SDK 1.1 起，依赖 ROS 2、SDK 或机器人硬件的项目代码在机器人开发板容器内构建、运行和测试。本地电脑用于编写代码、静态检查，以及运行数据下载、远程清理和离线分析等不依赖机器人运行环境的工具。
- 机器人型号为x2-ultra, 对应的urdf版本为v1.3.0

## 工作流程

1. 在本地工作区编写、修改代码和脚本。
2. Codex 默认只进行必要的本地文件检查、语法检查和代码审阅，不执行项目构建、运行或测试。
3. 由用户手动使用已配置的 rsync 同步任务，将本地改动推送至机器人开发板的对应工作区，并在开发容器内构建和测试。
4. 根据用户反馈的构建和测试结果继续修改本地代码，再由用户同步并验证。用户明确要求 Codex 执行远程验证时，再确认所需环境信息并执行。

用户操作步骤见根目录 [README.md](README.md)。涉及同步、下载或清理时，复用现有任务与脚本，未经任务要求不修改同步排除规则。

## 代码导航

以下路径均相对于工作区根目录。

- `docs/`：研究规划、设计评审与实验记录。[research_plan.md](docs/research_plan.md) 是阶段目标与状态入口；`underlay/docs/cn/_build/html/` 为随 SDK 提供的本地接口文档；查阅时核对 SDK、固件与机型适用范围。
- `docs/archive/`：历史设计、接口核实与实验记录。可用于追溯依据，但旧包名、命令、迁移方案和验证结果不直接作为当前用法或验收结论；当前入口优先查看对应包 README，并核对源码。
- `underlay/extra/`：独立参考工程与补充模型；`mc-rl/` 包含训练、仿真验证和部署参考，`x2_rl_deploy/` 提供 ONNX 策略部署与 MuJoCo 示例，`X2_URDF-v1.3.0/` 为补充官方模型。目录保留 `COLCON_IGNORE`，不纳入 Git、同步或构建，其中还有另一份 `aimdk_msgs`；当前厂商包与自研包不依赖此目录，不将其依赖、模型或真机结论直接套用到当前工程。
- `underlay/`：本地厂商 SDK 工作空间副本，包含厂商包、版本文件、接口清单及 SDK 文档；供 AI 查阅，不参与日常同步。机器人固定 underlay 为 `/home/agi/aimdk`。
- `overlay/`：本地自研工作空间，各包平铺于 `overlay/src/`；同步后在机器人 `/home/agi/aimdk-fsc/src/`，不增加 overlay 目录层级。
- `scripts/`：机器人容器内的 overlay 构建入口，以及不依赖 ROS 2 的本地工具，如远程清理脚本。
- `data/`：从机器人下载到本地的采集数据。
- `log/`：本地保存的机器人构建、调试和验证日志；容器内 colcon 也可能使用工作区同名目录。当前数据下载任务不下载日志。
- `exports/`：Codex 数据分析、日志调查等工作的输出结果。

| 分类 | 源码目录 → ROS 2 包名 | 职责与修改入口 |
| --- | --- | --- |
| 官方 SDK | `underlay/src/aimdk_msgs/` → `aimdk_msgs` | 厂商消息、服务与接口生成／预编译支持。接口定义在 `interface/`，构建逻辑在 `CMakeLists.txt` 和 `cmake/`；核对协议从这里开始。 |
| 第三方依赖 | `underlay/src/ruckig/` → `ruckig` | SDK 携带的在线轨迹生成库；业务代码按需依赖，不在其中放实验逻辑。 |
| 官方示例 | `underlay/src/examples/` → `examples`；`underlay/src/py_examples/` → `py_examples` | C++／Python SDK 调用示例，用于理解接口；自研功能分别进入下述自研包，不把示例模块作为公共库。 |
| 官方诊断工具 | `underlay/src/robotop/` → `robotop` | 官方 SDK 自带的终端状态、日志、通信与故障诊断工具，以及独立 Orin 资源监视节点；不承载自研采集或实验状态机。 |
| 自研实验工具 | `overlay/src/test/` → `aimdk_test` | 头部、手臂诊断与运动实验。入口为 `aimdk_test/head_test.py`、`arm_test.py`；`*_core.py` 为无 ROS 导入的校验／算法核心，`arm_test_motion.py` 为手臂 ROS 执行适配，`head_test_diagnostics.py` 提供实验 JSONL 日志。这里的 `test` 是业务包目录，不是全工作区单元测试目录。 |
| 自研功能命令 | `overlay/src/functions/` → `aimdk_functions` | 独立实用功能：`volume_control.py`（音量／静音）、`dual_arm_control.py`（URS 下单次关节增量／夹爪开度）。双臂纯计算与反馈校验在 `dual_arm_control_core.py`，日志在 `dual_arm_control_diagnostics.py`；不依赖测试包，也不作为所有公共代码的汇总包。 |
| 自研只读观测 | `overlay/src/locomotion_wrapper/` → `locomotion_wrapper` | 厂商关节状态聚合与标准 IMU 校验转发，不含运动控制或策略推理。节点在 `src/hardware_wrapper_node.cpp`，纯校验在 `include/locomotion_wrapper/state_validation.hpp`，配置在 `config/`，启动在 `launch/`，校验测试在 `test/`；`joint_watch.py` 仅监视标准关节状态。本轮不参与同步或构建。 |
| 自研数据采集 | `overlay/src/data_collection/` → `data_collection` | 独立订阅 SDK 双臂／夹爪反馈，生成统一 JSONL 帧并可选录制原始 MCAP，通过 ZeroMQ START／STOP／STATUS 控制录制。节点在 `src/data_collection_node.cpp`；`launch/data_collection.launch.py` 单独启动采集器，`robot_data_collection.launch.py` 为同包内的启动别名。 |

新增 `aimdk_test`／`aimdk_functions` 命令时，将模块放在相应 Python 包中，在各自 `setup.py` 的 `console_scripts` 注册，并在 `package.xml` 声明依赖；命令使用 ROS 2 包名，而非外层目录名，如 `ros2 run aimdk_test arm_test`。

## 关键约束

以下约束依据当前文档与源码，描述既有接口和行为；不代表已完成相应真机验收。修改相关代码时应保持这些约束；若任务明确要求改变契约，同步更新相关实现、验证与文档。

### 包边界与构建范围

- 自研业务不依赖官方示例作为公共库，不向 `aimdk_msgs`、`ruckig` 或 `robotop` 混入实验业务。公共逻辑先在实际使用的包内整理；不要仅为目录分类新建包或搬动源码。
- 自研包在机器人 Humble 容器内通过 `bash scripts/build_overlay.sh [包名...]` 构建，默认加载 `/home/agi/aimdk/install/setup.bash`，可由 `AIMDK_UNDERLAY_INSTALL` 指定实际安装目录。入口使用 `--base-paths src --packages-select`，仅允许 `aimdk_test`、`aimdk_functions`、`data_collection`，不构建厂商包或 `locomotion_wrapper`；不使用 `--packages-up-to` 将厂商依赖纳入构建。目录改名或依赖整理需同时核查入口、导入、日志路径探测和安装来源；仅推送代码不会清理受同步排除保护的旧安装产物。

### 上肢实验与反馈

- 运动入口默认只读，显式 `--execute` 才发布；保留原厂运控，使用原生上肢接口，不擅自改用 HAL、增加模式切换或自动返程。修改动作范围与限位逻辑前，核对对应包 README、SDK 定义和源码中的既有契约。
- 按关节名称映射反馈，按接口规定的单位与槽位构造完整目标；未选关节保持采样目标，不擅自裁剪。夹爪位置目标不等于力保持，空数组或模式 0 不视为通用保持。
- 保留反馈源时间、接收时效与时间推进检查，以及既有错误锁存和异常中止；不以本机时间掩盖冻结或回退，不擅自恢复已移除的 FSM 门槛或认定 raw=4 稳定。
- 模式切换须有界查询实际目标模式及 `RUNNING`；Set 成功回包、停止发布或请求默认站立均不能作为接管完成、急停或必达恢复的证明。
- 不将 HAL 字段视为原生上肢接口能力；使用 `effort` 估力前核实其物理含义、单位、符号和传动侧，位置偏置修正不宣称为通用重力力矩补偿。

实现细节见 [功能包说明](overlay/src/functions/README.md)、[实验包说明](overlay/src/test/README.md)；历史接口核实见 [上肢接口记录](docs/archive/upper_body_interface_findings.md)。

### 观测与采集语义

- 观测与采集保持只读，不触发运动或切换模式；采集器独立订阅 SDK 原始反馈，不依赖观测包。
- 配置必须匹配实际消息结构、话题、完整关节名称和传感器位置，不能仅改类型字符串或打开开关便宣称支持新输入；消费者按名称映射，不假定数组顺序。
- 区分源时间、接收时间和硬件同步；保留分析所需的原始错误、夹爪、模式和命令记录，不以标准消息替代。
- `START` 的 `ACK` 仅表示 writer 初始化成功；采集成功需以有效样本及读回验证为依据。数据目录须明确持久化位置和同步保护，不能假定工作区内 `rosbags/` 安全。

配置与数据格式细节见 [观测包说明](overlay/src/locomotion_wrapper/README.md)、[采集说明](overlay/src/data_collection/README.md)。

## Codex 执行要求

- 编写或修改脚本时，文件内的所有函数（包括类方法、构造函数和嵌套函数）都必须提供 docstring，说明其用途；有参数、返回值或重要异常时，也应说明其含义。Python docstring 应放在函数体的第一条语句位置，不能仅用普通注释代替。使用中文编写注释说明。
- 涉及 ROS 2、SDK 或机器人依赖的代码，应面向机器人开发板容器内的实际环境编写；不要仅凭本地安装了 Kilted 就引入 Kilted 专属接口或依赖。
- 不要将本地检查结果表述为机器人容器内构建或测试通过。汇报验证情况时，明确执行环境、已完成的检查和未执行的验证。
- 仅在任务确实需要远程连接、容器或挂载信息时，先检查项目现有配置和文档；仍无法确认时再向用户询问。不要因缺少这些信息而阻塞不依赖它们的代码编写和静态检查，也不要猜测执行命令或声称已完成验证。
- 本项目服务于科研，非开发项目。代码实现优先形成最小功能闭环，不考虑工程完整性。新增安全机制、保护检查或中止门槛前，先参考相近官方示例，结合接口要求、当前实验目标和已观察到的问题评估必要性。缺乏明确依据的稳定性、时序或精度阈值，优先记录观测结果，不默认作为阻断条件；根据实测需要逐步增加，并说明依据。已有明确约定的接口约束和异常处理，不因本原则自动移除。
