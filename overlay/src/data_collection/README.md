# data_collection 使用说明

版本 `0.3.0`，适用于机器人 Ubuntu 22.04／ROS 2 Humble、AimDK 1.1 和 X2 Ultra。双臂／夹爪静态录制及 MCAP 读回已验证；full 多模态接入已读回，但存在队列丢弃。新增全身关节来源、骨盆 IMU 与在线耗时统计待机器人容器验证。

## 功能与数据范围

按配置只读订阅所选来源，将每条收到的完整消息按来源频率保存到 MCAP。关节反馈、指令、IMU 和图像使用同一会话；在线录制不重采样、不插值，不生成统一帧 JSONL。START／STOP／STATUS 使用原有 ZeroMQ 控制协议。

来源目录为 [x2_sources.json](config/x2_sources.json)，包含消息类型、QoS 和图像关联内参：

| 来源 ID | 话题 | 类型 |
| --- | --- | --- |
| `arms` | `/aima/hal/joint/arm/state` | `aimdk_msgs/msg/JointStateArray` |
| `hands` | `/aima/hal/joint/hand/state` | `aimdk_msgs/msg/HandStateArray` |
| `head_joints` | `/aima/hal/joint/head/state` | `aimdk_msgs/msg/JointStateArray` |
| `waist` | `/aima/hal/joint/waist/state` | `aimdk_msgs/msg/JointStateArray` |
| `legs` | `/aima/hal/joint/leg/state` | `aimdk_msgs/msg/JointStateArray` |
| `arm_commands` | `/aima/hal/joint/arm/command` | `aimdk_msgs/msg/JointCommandArray` |
| `imu_chest` | `/aima/hal/imu/chest/state` | `sensor_msgs/msg/Imu` |
| `imu_pelvis` | `/aima/hal/imu/torso/state` | `sensor_msgs/msg/Imu` |
| `head_rgb`、`head_depth` | `/aima/hal/sensor/rgbd_head_front/rgb_image`、`depth_image` | `sensor_msgs/msg/Image` |
| `wrist_left_rgb`、`wrist_left_depth` | `/wrist/left/color/image_rect_raw`、`/wrist/left/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `wrist_right_rgb`、`wrist_right_depth` | `/wrist/right/color/image_rect_raw`、`/wrist/right/depth/image_rect_raw` | `sensor_msgs/msg/Image` |

骨盆 IMU 沿用现有[上肢反馈订阅](../functions/aimdk_functions/upper_body_ros.py)中的话题映射：
SDK 将 `/aima/hal/imu/torso/state` 标注为腰部 IMU，项目将其作为 pelvis IMU 使用，
现有实机记录的 `frame_id` 为 `base_link`。采集保留原始 `header`、姿态、角速度、线加速度及协方差，不转换坐标系。

每路图像自动附带 `<来源ID>_info`，类型为 `sensor_msgs/msg/CameraInfo`。头部对应 `rgb_camera_info`／`depth_camera_info`，腕部对应 `color/camera_info`／`depth/camera_info`。RGB 与深度可分别选择。扩展同类型来源时修改目录与预设；新增消息结构时补充显式类型适配。

关节来源依据 SDK [motocontrol 示例](../../../underlay/src/py_examples/py_examples/motocontrol.py)与[关节接口定义](../../../underlay/src/aimdk_msgs/interface/robot/hal/msg/JointStateArray.msg)。头部相机、IMU 与 HAL 接口依据 SDK 官方示例及消息定义；腕部设备绑定与话题沿用[相机使用说明](../functions/README.md#左右腕部-d405-与单帧图像)。头部连续图像、腕部深度和多模态并行录制仍需实机核实。原始关节数组顺序、名称、单位、错误和所有源时间字段保持原样；分析端按名称映射关节。

## 同步与构建

使用根目录说明中的现有同步任务，数据根目录及排除规则沿用[项目流程](../../../README.md)。以下命令在机器人开发容器工作区根目录执行。

### 构建前准备依赖

依赖声明见 [package.xml](package.xml)。`colcon build` 和 `source` 环境脚本均不会安装系统依赖。下面的清单以已有 SDK、ROS 2 Humble C++ 开发环境及可用的编译工具链为基础，包含本包直接使用的系统包及 rosbag2／MCAP 插件加载链需要的系统开发包：

| Ubuntu 22.04 系统包 | 用途／引入位置 |
| --- | --- |
| `pkg-config` | CMake 查找 ZeroMQ |
| `libzmq3-dev` | ZeroMQ C 库、`libzmq.pc` 及 C++ 接口 `zmq.hpp` |
| `nlohmann-json3-dev` | 来源配置与会话元数据；对应 rosdep 键 `nlohmann-json-dev` |
| `python3-zmq` | Python 控制客户端 |
| `libyaml-cpp-dev` | `rosbag2_storage → yaml_cpp_vendor → yaml-cpp` |
| `libzstd-dev` | `mcap_vendor → zstd_vendor → zstd`；同时提供头文件与链接库，仅装 `zstd` 命令行工具不足 |
| `libtinyxml2-dev` | `rosbag2_cpp／rosbag2_storage → pluginlib → tinyxml2_vendor` |
| `libconsole-bridge-dev` | `pluginlib → class_loader → console_bridge_vendor` |

已知机器人容器`/opt/ros`为只读挂载。历史apt失败、dpkg恢复过程及包名迁移依据见[历史排障记录](../../../docs/archive/data_collection_troubleshooting_20261026.md)。

在确认系统目录可写且包管理状态正常后，可在容器 root 终端一次准备以上 **Ubuntu 系统包**；普通用户使用相应的管理员权限。这条命令不列入 `ros-humble-*` 包：

```bash
apt-get update
apt-get install -y \
  pkg-config libzmq3-dev nlohmann-json3-dev python3-zmq \
  libyaml-cpp-dev libzstd-dev libtinyxml2-dev libconsole-bridge-dev
pkg-config --modversion libzmq
```

可在构建前一次查看以上包的安装状态；`install ok installed` 表示对应 apt 包已安装，不等同于构建或运行验收：

```bash
dpkg-query -W -f='${binary:Package}\t${Status}\t${Version}\n' \
  pkg-config libzmq3-dev nlohmann-json3-dev python3-zmq \
  libyaml-cpp-dev libzstd-dev libtinyxml2-dev libconsole-bridge-dev
```

本包直接依赖的 ROS 包为 `ament_cmake`、`rclcpp`、`rmw`、`builtin_interfaces`、`sensor_msgs`、SDK 的 `aimdk_msgs`、`rosbag2_cpp`、`rosbag2_storage`、`mcap_vendor`，以及运行时的 `launch`、`launch_ros`、`ament_index_python`、`rosbag2_storage_mcap`、`rosbag2_py`、`rclpy`、`rosidl_runtime_py`。标准 apt 安装会递归安装 ROS 包声明的依赖；仅有 `/opt/ros/humble/share/<包名>` 或 CMake 配置文件，不能证明对应系统开发包齐全。

当前机器人容器先复用 SDK 已提供的 ROS 文件，补齐其需要的 Ubuntu 系统开发包；`aimdk_msgs` 继续使用 SDK 工作区提供的版本。若确认某个 ROS 包实际缺失，应先核对 SDK 容器的挂载、版本及厂家支持的安装方式，再决定如何补齐。仅在 ROS 安装路径可写、由 apt 管理且与 SDK 兼容的环境中，才适用标准 `ros-humble-*` apt 安装流程。

MCAP 是当前录制格式。Humble 官方 `mcap_vendor` 将 LZ4 源码编入自身库，上述链路不额外要求系统 `liblz4-dev`；SQLite 存储插件也不是当前 MCAP 录制所需依赖。依据：[Humble MCAP 构建定义](https://github.com/ros2/rosbag2/blob/humble/mcap_vendor/CMakeLists.txt)、[pluginlib 依赖](https://github.com/ros/pluginlib/blob/humble/pluginlib/package.xml)、[class_loader 依赖](https://github.com/ros/class_loader/blob/humble/package.xml)、[rosdep 系统包映射](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml)。实际容器版本及缺包情况需在容器内核查，以上清单尚未经机器人容器构建验收。

### 构建

统一入口启动腕部驱动时复用 `aimdk_functions` 已安装的 `config/wrist_cameras.json`，需要 `realsense2_camera`。同步后构建：

```bash
bash scripts/build_overlay.sh aimdk_functions data_collection
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
```

## 统一启动与来源选择

数据根目录使用 `/home/agi/aimdk-fsc/data`，由项目目录挂载持久化并受同步 `/data/` 排除规则保护。远程清理任务仍会按其既有规则清除此目录。

默认只采双臂和夹爪，使用 `profile` 选择 [collection_profiles.json](config/collection_profiles.json) 中的组合：

| 预设 | 内容 |
| --- | --- |
| `upper_body`（默认） | 双臂、夹爪反馈 |
| `whole_body_imu` | 头部、双臂、腰部、双腿、夹爪反馈，以及胸部、骨盆 IMU；共 7 个话题 |
| `manipulation` | 双臂、夹爪、HAL 手臂指令、头部 RGBD、双腕 RGB |
| `camera_check` | 头部 RGBD、双腕 RGB |
| `full` | 全部关节反馈、HAL 手臂指令、胸部及骨盆 IMU、头部与双腕 RGBD，以及图像内参 |

```bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/aimdk-fsc/data profile:=manipulation
```

采集全身关节反馈及胸部、骨盆 IMU，使用：

```bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/aimdk-fsc/data profile:=whole_body_imu
```

该预设不创建相机订阅或启动腕部驱动，按原频率保存七路反馈。录制前同步并重新构建 `data_collection`，加载 overlay 环境，再沿用 START／STATUS／STOP。头部、腰部和双腿仍需核对真机实际发布者与完整关节名称；按原始消息保留全部关节，不硬编码数量或数组槽位。所选来源未收到消息时按既有规则报告缺流，不自动跳过。

无图像会显著降低输入负载，零丢弃仍以本轮 STATUS、收尾计数和 MCAP 读回为依据。STOP 后核对七路均有数据、submitted=written、dropped=0、队列排空，并核对原始关节名称、状态／故障、源时间和 IMU 字段。完整性标记的含义见下文会话状态说明。

仅录骨盆 IMU 时使用：

```bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/aimdk-fsc/data streams:=imu_pelvis
```

也可用逗号分隔的 `streams` 覆盖预设。例如只录左腕 RGB 和双臂反馈：

```bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/aimdk-fsc/data streams:=arms,wrist_left_rgb
```

入口自动添加关联内参，仅启动所选腕部设备及其图像流。设备序列号、命名空间与图像参数读取现有腕部配置；来源目录中的话题应与该配置保持一致。头部 RGBD、关节和 IMU 复用机器人已有发布者。已经启动腕部驱动时增加 `start_wrist_cameras:=false`，复用其输出。

配置在启动时冻结。切换采集范围时先 STOP，再关闭并重启统一入口。停止录制保持相机运行；关闭启动入口结束由该入口启动的驱动和采集器。

| 参数 | 默认值／用途 |
| --- | --- |
| `data_root` | 必填，持久化根目录绝对路径 |
| `profile`／`streams` | `upper_body`／空；非空 streams 覆盖预设 |
| `mapping_file` | 安装后的 `config/x2_sources.json` 来源目录 |
| `profiles_file` | 安装后的 `config/collection_profiles.json` |
| `start_wrist_cameras` | `true`；只启动选中的腕部设备，false 复用已运行驱动 |
| `mcap_storage_config` | 安装后的 `config/mcap_storage.yaml` |
| `queue_capacity` | `512` 条，满时丢弃新消息并按话题计数 |
| `zmq_bind_address`／`zmq_port` | `127.0.0.1`／`5555` |
| `robot_model`／`firmware` | `x2-ultra`／`unknown`，保存实际环境信息 |

## 会话控制与状态

启动后等待控制指令。另一个控制终端加载相同环境，再执行：

```bash
ros2 run data_collection collection_control START
ros2 run data_collection collection_control STATUS
ros2 run data_collection collection_control STOP
```

START／STOP 成功返回 ACK，失败返回 ERROR。录制中重复 START 保持原会话；正常停止后重复 STOP 幂等。START 的 ACK 表示存储初始化完成，需查看 STATUS 确认各来源实际有数据。STOP 关闭接收门、排空队列、关闭 MCAP 并保存摘要后返回；节点继续等待下一次 START。

STATUS 包含 `recording_enabled`、会话目录、队列条数／序列化消息字节数及峰值。`raw` 按话题包含 submitted／written／dropped、接收字节数、首末接收时间、首末源时间、实际接收频率和最近消息描述；在线状态额外提供单调接收年龄。图像描述保存分辨率、编码、步长和大小端标记。字节统计仅计序列化消息，不包含队列对象、DDS 缓冲和压缩器内存；队列容量仍按条数限制。

源时间为零、重复、回退，以及 SDK 测量时间／序号异常均计数。保留源时间，不替换为本机时间；单调时钟仅用于接收频率与年龄。MCAP 的消息记录时间使用回调入口 ROS 接收时间，原消息保存其源时间。这些时间不构成硬件同步证明，跨设备配对在离线分析中完成。

`complete` 表示正常关闭且无存储错误；`data_complete` 还要求每个选中话题至少成功写入一条消息且无队列丢弃。时间异常计数另行评估；该标记不包含字段有效性、同步精度或样本数量验收。未选来源不计为缺流。

控制客户端默认连接 `tcp://127.0.0.1:5555`，超时 5000 ms，可用 `--endpoint`／`--timeout-ms` 调整。STOP 超时后继续查询 STATUS，确认 `recording_enabled=false`、`closed=true`、`queue_pending=0`。Python 客户端需使用可导入 `zmq` 的解释器。

## 存储与读回

每次 START 按开始记录时的北京时间创建 `YYYYMMDD_HHMMSS/`，如 `20261117_143025/`。同目录重名时追加 `_1`、`_2` 等递增序号；历史目录名保留。

| 文件 | 内容 |
| --- | --- |
| `raw/*.mcap`、`raw/metadata.yaml` | 所选话题的完整 ROS 消息及 rosbag2 元数据 |
| `session.json` | schema_version=2，生效来源配置、环境、计数及收尾结果 |
| `mcap_storage.yaml` | 本次会话实际使用的容器压缩配置快照 |

默认 Zstd／Fast 分块无损压缩，保留消息索引与摘要。RGB 和深度均保存原始 Image，不进行 JPEG 或视频编码。MCAP 可在压缩无收益时保存未压缩块；配置依据 [Humble MCAP 插件](https://github.com/ros2/rosbag2/blob/humble/rosbag2_storage_mcap/README.md#writer-configuration)。实际体积、持续写入速度和 Jetson 负载需录制测量。

STOP 后用 STATUS 中的实际会话目录替换下方占位符：

```bash
ros2 run data_collection inspect_collection /ABSOLUTE/SESSION_DIR --verify-mcap
```

工具遍历 MCAP、反序列化所有消息，核对话题、类型及逐话题写入数，检查图像尺寸、步长与数据长度，并报告源时间重复和回退。退出码 0 表示当前会话完整性规则通过，1 表示不完整，2 表示读取、结构或摘要不一致。仍需抽查图像内容、关节名称和实际数据意义。

不带 `--verify-mcap` 时仅用标准库检查摘要，新会话不读 MCAP，报告 `raw_mcap_verified=false`；可在本地使用。历史 schema_version=1 会话继续按原规则读取 `frames.jsonl`；历史原始 MCAP 也可在 ROS 环境使用上述读回选项。旧 `record_raw_mcap` 启动参数已移除，新录制始终保存 MCAP。

## 压缩与写盘耗时诊断

在机器人 Humble 容器内重新构建 `data_collection` 后，按原命令录制并 STOP，`session.json` 与 STATUS 自动包含 `timing`。统计只累计成功完成的阶段，单位为纳秒，不改变图像、压缩或队列配置：

| 字段 | 含义 |
| --- | --- |
| `serialization` | 回调内完整消息序列化的墙钟耗时，含后续入队被丢弃的消息 |
| `queue_wait` | 消息成功入队至工作线程取出的等待时间 |
| `write_call` | rosbag2 Writer::write 调用的墙钟与当前线程 CPU 耗时 |
| `write_by_topic` | 按触发该次 write 调用的话题累计耗时 |
| `close_call` | Writer::close 调用，含尾块、索引及元数据收尾 |
| `stop_drain_wall_ns` | STOP 通知后等待写入线程排空、退出的墙钟耗时 |

每项包含调用数、累计墙钟、平均和最大墙钟；只有 write／close 统计线程 CPU，其余 `total_cpu_ns=null`。queue_wait 为逐消息等待的累计值，不能当作 CPU 消耗或与会话时长直接比较。MCAP 块可能由不同话题共同构成，因此 write_by_topic 表示触发压缩／写入的调用来源，不是该话题独占的压缩开销。累计统计不提供逐调用时间线或分位数。

write 调用内部同时包含 MCAP 缓冲、块压缩和文件写入；close 也不代表执行了 fsync。墙钟与线程 CPU 的差值可能包含 I/O 等待或线程调度，不能直接等同于磁盘耗时。

为分别测量压缩、文件 write 和 fsync，停止录制后在**同一机器人、同一数据磁盘**执行离线基准。它复用已录 MCAP 的原始块，无需 ROS 消息类型，不订阅话题：

```bash
ros2 run data_collection benchmark_collection_io \
  /home/agi/aimdk-fsc/data/20261117_143025
```

也可直接运行同步后的脚本，无需为该脚本单独构建：

```bash
python3 src/data_collection/scripts/benchmark_collection_io.py \
  /home/agi/aimdk-fsc/data/20261117_143025
```

默认测量全部块；首次快速检查可增加 `--max-chunks 100`。`--zstd-level` 默认 `-3`，对应 MCAP Fast 的数值级别，依据 [MCAP 压缩级别映射](https://github.com/foxglove/mcap/blob/releases/cpp/v1.1.0/cpp/mcap/include/mcap/writer.inl#L195)。脚本从系统 libzstd 读取版本并写入结果，当前不自动替换录制配置。

流程为逐块读取并解压，在计时区间外分配缓冲，再分别计时 ZSTD_compress2、文件 write，最后单独测量一次 fsync。文件 write 可能只写入页缓存；fsync 耗时单列，并计入汇总写入吞吐。读取、解压、内存分配、诊断日志和 MCAP 索引处理不计入阶段吞吐；读源文件与诊断文件仍会竞争同一磁盘。此结果是离线基准，不是原录制过程中压缩和 I/O 的精确拆分，也不是纯磁盘设备性能测试。

输出默认为来源会话内的 `YYYYMMDD_HHMMSS/`（基准开始时的北京时间，重名追加短序号），包含 `chunks.jsonl` 和 `summary.json`，随现有数据下载流程取回。临时文件也放在该目录的文件系统，结束时自动删除；测试期间需要容纳一次重新压缩输出的空间。可用 `--output-dir` 指定新的结果目录及目标写入磁盘，原始 MCAP 保持不变。不要与正式录制同时运行此基准。

结果重点查看 compression_input_MB_per_s、write_output_MB_per_s_including_final_fsync、combined_stage_input_MB_per_s，以及阶段 CPU／墙钟耗时。原始输入吞吐与压缩后输出吞吐的计量基准不同，比较时使用相应字节数；本次 full 仅图像约 230.4 MB/s，可作为负载量级参考，不新增通过／中止门槛。

本轮只完成本地语法与静态检查，未在本地或机器人执行基准，未构建／运行带计时的采集节点。建议取回一段新 full 会话的 session.json 及离线基准 summary.json，再结合队列丢弃判断瓶颈。

## 容器验收

1. 使用 whole_body_imu：确认七个预期话题均有数据，包含 `/aima/hal/imu/chest/state` 和 `/aima/hal/imu/torso/state`，头部／腰部／双腿关节完整名称可读回，反馈／IMU 数值与原始状态保留，逐话题无丢弃且正常收尾。
2. 默认预设录制：确认仅双臂／夹爪两个话题，原始关节名称、错误及时间字段读回一致，新目录不生成 frames.jsonl。
3. 单独选择 `wrist_left_rgb`：确认仅左腕彩色流及内参写入，右腕不由本入口启动，独立相机会话能正常 STOP 和读回。
4. 使用 `manipulation`，再复用已启动驱动：确认头部 RGBD、双腕 RGB、反馈及指令逐话题有数据，内参存在；抽查图像与深度编码及标定内容。
5. 全量短录制：记录时长、MCAP 体积、实际接收频率、队列峰值、丢弃和 Jetson 负载；检查深度原始数值与容器压缩配置。
6. 重复 START、重复 STOP 及连续两次会话：确认幂等、收尾计数守恒、队列清空且消息不跨会话。已选来源不发布时如实报告无数据。

C++ 节点、MCAP 写入和 ROS 启动均由用户同步后在机器人容器构建、运行与验证。本地旧 JSONL 验证结果不代表本版多模态验收通过。
