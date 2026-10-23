# WBC 部分迁移结果与主代理验收

执行依据：`MIGRATION_PLAN.md`。三个子代理分别负责观测、采集与离线工具、只读策略；主代理审阅并提出修改后统一构建和验收。旧仓库 `/home/gary/projects/WBC_middleware` 保持未修改，`git status --short` 仍为空。

## 当前结论

观测和只读策略两个新包已在新 SDK / ROS 2 Kilted / Python 3.12 下构建、安装并通过测试。采集包与工具已迁入，工具的实际 MCAP 往返检查通过；**采集 C++ 节点仍因缺少系统 ZeroMQ 开发文件而未通过构建验收**，不能把该模块视为已可启动。

新 SDK 原有接口检查继续全部通过。没有连接机器人，没有执行运动模式切换或动作下发；没有迁入旧 `aimdk_msgs`、Ruckig 副本、旧 SDK 示例、旧 `test.py`、运动服务代理。

## 交付位置

| 位置 | 迁移内容与修改 |
| --- | --- |
| `src/locomotion_wrapper` | 只读关节 / IMU 观测；默认验证完整的 12 个腿关节；保留源时间；源时间和 monotonic 接收时效、错误状态、数值与组时间差检查；失效停止对应状态输出 |
| `src/locomotion_wrapper/config/wbc_observer.yaml` | 腿部专用配置；未核实的头臂腰组不接入该 profile；IMU 保留旧观测节点的胸部输入 |
| `src/data_collection_edge_agent` | MCAP 采集节点、组合 launch；显式 rosbag2 metadata、join 线程、错误响应、唯一目录名、默认 loopback ZMQ 控制 |
| `tools/edge_agent_control.py` | START / STOP 命令行控制，超时与 ERROR 处理；交互会话退出时尝试 STOP |
| `tools/mcap_to_robot_csv.py` | 保留关节名字和 position / velocity / effort、源时间、收包时间、IMU 四元数、对齐误差；不依赖 pandas |
| `src/wbc_middleware` | ROS Python 影子推理包、模型、观测配置、原 metadata；只有状态订阅与日志，没有执行器出口 |
| `requirements-migration.txt` | 本机新增 Python 依赖；继承原 runtime 清单并约束 NumPy <2 |

观测节点的 2ms 定时器只检查新缓存，未更新的缓存不持续重发；多组聚合保留最旧源时间。它是多个时刻的近同时快照，不能替代硬件同步。标准 `JointState` 无法保留厂商每组全部时间 / 状态字段，相关研究应另录原始厂商话题。

CSV 使用按时间顺序的一对一容差配对，保留未配对样本和重复时间戳；不复用样本，不宣称最近邻最优匹配或硬件同步。所有选中样本会进入内存，超大录包需要分段。可选 rosbags 后端未安装、未测试；本轮使用系统已有 rosbag2_py 验证。

## 关键接口判断

旧 SDK 56 个定义，新 SDK 121 个定义；45 个共同定义去除注释后相同，8 个发生变化，3 个旧定义移除。`JointStateArray` 内嵌的 `JointState` 已由温度 / 电压字段变为 `error_code`，所以源码使用相同字段并不意味着旧消息二进制可复用。迁移包均只链接目标新 SDK。

两个原项目的 Ruckig 源文件完全一致，因此未迁入第二份。旧 `PlayMediaFile`、`LedStripCommand` 已移除，旧运动代理与示例不整体覆盖新 SDK。

## 策略的有效范围

真实模型输入：`obs=[batch,47]`，`hidden_state/cell_state=[1,batch,64]`；输出 `actions=[batch,12]` 和两个 `[1,batch,64]` 循环状态，均为 float32。本包固定 batch=1，检查名称、形状、类型和有限值。

安装后的模型与旧仓库 SHA256 一致：

`43e0eb74582d58c248e6c8797865b3d239518b84f5e3de20ee04b4c8247a7cc8`

模型自身 metadata 标记 `validation_only=true`，且把部署 YAML 指定为 runtime source of truth。旧仓库没有提供相应训练部署 YAML。这是保持影子推理的重要依据，不能把推理通过解释成模型已适合真机控制。

保留旧脚本的关节顺序、默认位姿、47 维观测顺序、xyzw 重力公式、50Hz 节奏和每步 +0.02 rad 相位规则。单位四元数的重力结果为 [0,0,-1]。是否与训练一致尚待训练配置 / 参考 rollout 核对；未擅自添加裁剪或控制增益。完整、新鲜、时间相容的 torso IMU 与 12 腿关节数据齐备后才推理；断流、错误、时间回退或调度中断后清空缓存与 LSTM。

观测 / 采集默认胸部 IMU，而策略默认 torso IMU，两者位置和坐标系不同；录包回放用于模型验证时，必须明确配置所需 IMU，不能仅重命名话题替代外参核验。

## 验收证据

- 主代理统一构建 `locomotion_wrapper`、`wbc_middleware` 成功，观测包 `BUILD_TESTING=ON`，安装资源和两个策略入口可解析。
- 观测核心 **8 个 gtest** 通过；colcon 汇总额外计入一个 CTest 包装项，不重复视为第 9 个业务测试。
- 策略 **20 个 pytest** 通过，含真实 ONNX 两步推理、观测排列、完整性、NaN、错误、时钟冻结 / 回退、断流和恢复。
- 策略 + 工具联合执行结果 **27 passed, 1 skipped**；工具包含真实 MCAP 写入、读取、CSV 导出检查。ZMQ IPC 实际套接字测试因沙箱 `EPERM` 跳过，不能视作在线控制通道已验收。
- 修复策略 CLI 帮助在初始化 ROS 前退出；安装后的 `shadow_policy --help` 和观测 launch `--show-args` 通过。
- 新 SDK 原有 88 个消息、33 个服务序列化检查、38 个 Python 示例入口、2 个 robotop 入口继续通过；原 37 个 C++ 示例无缺失动态库。
- 新迁入代码检查未发现在关节命令类型或运动服务上创建发布器 / 客户端。状态 observer 的标准传感器发布器是预期行为。
- 未验证真实 ROS 跨机发现、机器人数据率、实时性、机型外参或任何真机控制行为。原 SDK 已记录的格式检查历史失败不属于本轮已修复项。

日志：`log/checks/migration/`、`migration-colcon-tests.txt`、`migration-python-tests.txt`、`migration-python-tests.xml`、`post-migration-sdk-check.txt`、`observer-launch-args.txt`、`shadow-policy-help.txt`。

## 环境与剩余验收项

本机虚拟环境用 uv 从可用本地缓存安装 `onnxruntime==1.24.4`、`pyzmq==27.1.0` 以及所需 flatbuffers / protobuf；保持系统 NumPy 1.26.4。在线安装尝试因 DNS 失败而未成功，未修改系统软件包。

采集节点缺少 `libzmq.pc`、`zmq.h`、`zmq.hpp`，仅存在运行时 `libzmq.so.5`。已向用户请求在本机安装：

```bash
sudo apt install libzmq3-dev cppzmq-dev
```

安装后必须继续构建并验收该 C++ 节点，当前不将它标记为通过：

```bash
source /opt/ros/kilted/setup.bash
source install/local_setup.bash
source .venv/bin/activate
python -m colcon build --base-paths src --packages-select data_collection_edge_agent
```

当前将整个 `src` 一次性构建也会受到此缺失依赖影响；已验收两个包可用 `--packages-select locomotion_wrapper wbc_middleware` 构建。没有通过隐藏包或伪造开发头文件掩盖阻塞。

`requirements-migration.txt` 是本机 Python 3.12 已验证环境；ONNX Runtime 1.24 需要 Python ≥3.11。迁往 Humble / Python 3.10 开发板时，要重新选择支持其 Python 和 ARM64 的版本，不要复制本机 `.venv`、`build`、`install` 或 x86_64 预编译缓存。

## 使用已验收模块

从项目根目录加载环境后，分别启动所需只读功能：

```bash
source /opt/ros/kilted/setup.bash
source install/local_setup.bash
source .venv/bin/activate
ros2 launch locomotion_wrapper observer.launch.py
# 另一终端：同样加载环境后，只读策略输出仅进入日志
ros2 run wbc_middleware shadow_policy
```

这些启动命令留给有合适网络权限的运行环境，本轮未在机器人上执行。完整采集链路命令及工具参数见 `src/data_collection_edge_agent/README.md`，须先完成剩余依赖和构建验收。

下一阶段应先做隔离回放 / 仿真和源时钟核验，然后做真机纯订阅检查。真实控制出口、控制权接管、限幅和停止策略需要单独设计和验证，不在本轮迁移范围。
