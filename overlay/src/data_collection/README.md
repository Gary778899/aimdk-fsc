# data_collection 使用说明

版本 `0.3.0`，适用于机器人 Ubuntu 22.04／ROS 2 Humble、AimDK 1.1 和 X2 Ultra。双臂／夹爪静态录制及 MCAP 读回已验证；full 多模态接入已读回，但存在队列丢弃。新增全身关节反馈与指令来源、骨盆 IMU 及在线耗时统计待机器人容器验证。

## 常用采集流程

在机器人 Humble 容器的终端 A 和 B 分别加载环境。首次部署或同步代码更新后，先按[同步与构建](#同步与构建)准备对应的包。

```bash
cd /home/agi/aimdk-fsc
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
```

| 目的 | 操作入口 | 主要产物 |
| --- | --- | --- |
| 只采机器人数据 | [启动选流](#统一启动与来源选择) → [手动起停和读回](#手动起停与读回) | MCAP、`session.json` |
| 单独验收 PICO 输入 | [独立 PICO 输入采集](#独立-pico-输入采集) | 原始 XR MCAP、`xr.jsonl`、对账报告，可在无夹爪时使用 |
| 录制完整的遥操试次 | [单次示教录制与读回](#单次示教录制与读回) | MCAP、XR 与控制日志、实验清单、读回报告 |

### 统一启动与来源选择

终端 A 启动采集器并保持运行。以下为全身反馈和 IMU 的无图像录制。

```bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/aimdk-fsc/data profile:=whole_body_imu
```

用 `profile` 选择预设，或用逗号分隔的 `streams` 覆盖预设。只选预计有数据的来源，任一已选话题零样本都会影响完整性。以下只采双臂反馈与左腕彩色图像，在终端 A 加载 RealSense 环境后启动。 [预设配置](config/collection_profiles.json)

```bash
source /home/agi/aimdk-fsc/realsense_ws/install/local_setup.bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/aimdk-fsc/data streams:=arms,wrist_left_rgb
```

| `profile` | 采集内容 |
| --- | --- |
| `upper_body`（默认） | 双臂、夹爪反馈及胸部、骨盆 IMU |
| `whole_body_imu` | 头部、双臂、腰、双腿、夹爪反馈及胸部、骨盆 IMU |
| `whole_body_commands` | 头部、双臂、腰、双腿、夹爪的 HAL 指令 |
| `manipulation` | 双臂、夹爪反馈、HAL 手臂指令、头部 RGBD、双腕 RGB |
| `camera_check` | 头部 RGBD、双腕 RGB |
| `full-no-cam` | 全身关节及夹爪反馈、对应 HAL 指令、胸部和骨盆 IMU |
| `full` | `full-no-cam` 加头部及双腕 RGBD |

图像自动附带内参。入口默认只启动选中的腕部驱动，已有驱动时加 `start_wrist_cameras:=false`。更换来源时先 STOP，再关闭终端 A 的 launch 并按新参数启动。保持同一选流时可连续录制多次。全部来源 ID 见[功能与数据范围](#功能与数据范围)。

### 手动起停与读回

终端 B 在采集器就绪后开始录制。收到 START 的 `ACK` 后执行本轮实验，其间用 STATUS 查看各来源是否收到数据及队列丢弃数。

```bash
ros2 run data_collection collection_control START
ros2 run data_collection collection_control STATUS
```

结束时先停止录制。

```bash
ros2 run data_collection collection_control STOP
```

收到 `ACK` 后，从 STATUS 取实际会话路径并读回。STOP 超时时按[会话控制与状态](#会话控制与状态)确认关闭后再读回。

```bash
SESSION=$(ros2 run data_collection collection_control STATUS | python3 -c 'import json, sys; print(json.load(sys.stdin)["session_dir"])')
ros2 run data_collection inspect_collection "$SESSION" --verify-mcap
```

检查各选中话题有样本、`submitted=written`、`dropped=0`，并核对 MCAP 读回结果。

### 独立 PICO 输入采集

本流程仅使用 PICO 输入，可在未安装夹爪时验收 XR JSONL 与 MCAP 的对账。只需要 XR JSONL 时直接使用独立录制命令。 [固定时长录制与发布](../remote_control/README.md#固定时长录制与发布)

终端 A 选择 `xr_raw`。已运行采集器时，先按上文结束旧录制及 launch，再启动。

```bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/aimdk-fsc/data/xr_baseline streams:=xr_raw
```

终端 B 确认 XR Service 运行，连接 PICO 并开启 Controller 和 Send。服务就绪后发送 START，收到 `ACK` 再取得本次会话路径并录制 30 秒。设备 ID 按实际 SDK 回调填写。 [PICO 与 XR Service](../remote_control/README.md#pico-与-xr-service)

```bash
ros2 run data_collection collection_control START
```

```bash
SESSION=$(ros2 run data_collection collection_control STATUS | python3 -c 'import json, sys; print(json.load(sys.stdin)["session_dir"])')
ros2 run remote_control xr_record --publish-xr-raw \
  --device-id TestDevice --input-config /home/agi/aimdk-fsc/src/remote_control/config/input.json \
  --sdk-library /opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so \
  --output "$SESSION/xr.jsonl" --duration 30
```

等 `xr_record` 返回，让 SDK 退出时排空的回调也进入录制窗口。发送 STOP，确认 MCAP 关闭后对账。

```bash
ros2 run data_collection collection_control STOP
```

```bash
python3 scripts/record_trial.py verify "$SESSION"
```

默认使用会话目录内的 `xr.jsonl`。日志在其他位置时给 `verify` 加 `--xr-jsonl "$XR_JSONL"`，其中 `XR_JSONL` 为同次录制的实际路径。查看 `readback.json` 中的完整双手帧及对账计数。要下载此类独立会话或普通机器人采集，请在本地 VS Code 下载任务中选 `all`。

### 单次示教录制与读回

**终端 A 启动采集器。** 以下为任务 `teleop_demo` 的无图像配置，显式选择 `xr_raw`。可按任务增减机器人话题。

```bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/aimdk-fsc/data/experiments/teleop_demo \
  streams:=arms,hands,arm_commands,hand_commands,xr_raw
```

**终端 B 准备 PICO 和试次条件。** 确认 XR Service 运行，连接 PICO 并开启 Head、Controller、Send，开始前松开两侧 Grip。只读和执行均需要新鲜的机器人反馈、HAL 双臂位置命令及两侧有效 `CLAW` 反馈。无夹爪时可走上面的独立 XR 流程。 [PICO 与 XR Service](../remote_control/README.md#pico-与-xr-service)

以下为双侧 CLAW 空载、关闭位置偏置补偿的示例。每轮按实际工况更新重复编号、起始姿态或参考编号、工具与负载，并使条件与遥操参数一致。设备 ID 使用 PICO SDK 回调中的实际标识。

```bash
CONDITIONS=$(mktemp /tmp/teleop_demo_conditions.XXXXXX.json)
cat > "$CONDITIONS" <<'EOF'
{
  "protocol": "teleop_demo",
  "group": "compensation-off",
  "repetition": 1,
  "initial_condition": "双臂静止于本轮接管起始姿态，两侧 Grip 松开",
  "tool_and_load": "双侧 CLAW，空载",
  "compensation": false
}
EOF

TELEOP_ARGS=(
  --device-id TestDevice
  --input-config /home/agi/aimdk-fsc/src/remote_control/config/input.json
  --mapping-config /home/agi/aimdk-fsc/src/remote_control/config/mapping.json
  --sdk-library /opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so
  --load-description '双侧 CLAW，空载'
  --no-compensation
)
```

**二选一开始录制。** 只读模式记录输入、反馈与 IK 结果。

```bash
python3 scripts/record_trial.py record --conditions "$CONDITIONS" -- "${TELEOP_ARGS[@]}"
```

实际执行前，确认动作空间并结束其他上肢发布进程。在终端 B 请求并查询 URS，确认 `UPPERBODY_REMOTE_SPLIT` 和 `RUNNING` 后再运行下方命令。终端 A 保持采集器运行。 [URS 模式控制](../functions/README.md#urs-模式控制)

```bash
python3 scripts/record_trial.py record --conditions "$CONDITIONS" -- "${TELEOP_ARGS[@]}" --execute
```

等待首次 Grip 提示后，按住对应侧 Grip 接管，Trigger 控制该侧夹爪。 [Grip 与执行行为](../functions/README.md#遥操持续执行)

**在终端 B 按 Ctrl-C 结束，等命令返回。** 该入口负责 START，退出时先等遥操退出并收尾日志，再发 STOP。Ctrl-C 返回 `130`，采集收尾结果见 `experiment.json` 的 `capture.state`。采集器留在终端 A 等待下一轮。每次录制使用新的会话目录。

本轮执行结束后，可在终端 B 按模式控制步骤请求恢复 SD 并查询 `STAND_DEFAULT` 和 `RUNNING`。如需在 URS 下回位，先完成回位再恢复 SD。 [URS 模式控制](../functions/README.md#urs-模式控制)

**读回与标注。** 将入口打印的 `会话目录` 绝对路径粘贴给下方提示。它位于 `/home/agi/aimdk-fsc/data/experiments/teleop_demo/` 下。

```bash
read -r -p '粘贴本次会话目录的绝对路径: ' SESSION
python3 scripts/record_trial.py verify "$SESSION"
```

用 `readback.json` 核对采集完整性、MCAP 读回与 XR 对账。只读试次若缺少选中的 HAL 指令话题，会报告采集不完整。各字段见 [XR 对账与时间配对](#xr-对账与时间配对)。

同一会话保存 `experiment.json`、`session.json`、`raw/`、`mcap_storage.yaml`、`xr.jsonl`、`control/` 和 `console.txt`。现场备注可写入 `notes.md`。按实际结果选择 `success`、`failure` 或 `aborted`，可用 `--stage` 记录任务阶段。未标注的试次为 `unlabeled`。

```bash
python3 scripts/record_trial.py label "$SESSION" --outcome success --reason '双臂示教完成，夹爪按计划开合'
```

**下载。** 在终端 B 生成会话相对路径。再到本地 VS Code 运行 **机器人：下载采集数据**，选择本次使用的机器人连接并填入该路径。整次试次将下载到本地 `data/experiments/` 下的对应目录。按会话下载要求实验清单确认采集已收尾。

```bash
printf '%s\n' "${SESSION#/home/agi/aimdk-fsc/data/experiments/}"
```

## 功能与数据范围

按配置只读订阅关节反馈、HAL 指令、IMU、相机等来源，按各路接收频率将完整消息保存到 MCAP，跨来源配对留给离线分析。选中 `xr_raw` 时，同一会话也保存 XR 发布端提供的原始回调。采集器通过 ZeroMQ 处理 START／STOP／STATUS，记录逐流计数与丢弃，并提供通用 MCAP 读回。XR 协议和输入质量由 `remote_control` 解释，试次入口汇总采集与 XR 核验结果。

来源目录为 [x2_sources.json](config/x2_sources.json)，包含消息类型、QoS 和图像关联内参：

| 来源 ID | 话题 | 类型 |
| --- | --- | --- |
| `xr_raw` | `/remote_control/xr/raw` | `std_msgs/msg/String` |
| `arms` | `/aima/hal/joint/arm/state` | `aimdk_msgs/msg/JointStateArray` |
| `hands` | `/aima/hal/joint/hand/state` | `aimdk_msgs/msg/HandStateArray` |
| `head_joints` | `/aima/hal/joint/head/state` | `aimdk_msgs/msg/JointStateArray` |
| `waist` | `/aima/hal/joint/waist/state` | `aimdk_msgs/msg/JointStateArray` |
| `legs` | `/aima/hal/joint/leg/state` | `aimdk_msgs/msg/JointStateArray` |
| `arm_commands` | `/aima/hal/joint/arm/command` | `aimdk_msgs/msg/JointCommandArray` |
| `head_commands` | `/aima/hal/joint/head/command` | `aimdk_msgs/msg/JointCommandArray` |
| `waist_commands` | `/aima/hal/joint/waist/command` | `aimdk_msgs/msg/JointCommandArray` |
| `leg_commands` | `/aima/hal/joint/leg/command` | `aimdk_msgs/msg/JointCommandArray` |
| `hand_commands` | `/aima/hal/joint/hand/command` | `aimdk_msgs/msg/HandCommandArray` |
| `imu_chest` | `/aima/hal/imu/chest/state` | `sensor_msgs/msg/Imu` |
| `imu_pelvis` | `/aima/hal/imu/torso/state` | `sensor_msgs/msg/Imu` |
| `head_rgb`、`head_depth` | `/aima/hal/sensor/rgbd_head_front/rgb_image`、`depth_image` | `sensor_msgs/msg/Image` |
| `wrist_left_rgb`、`wrist_left_depth` | `/wrist/left/color/image_rect_raw`、`/wrist/left/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `wrist_right_rgb`、`wrist_right_depth` | `/wrist/right/color/image_rect_raw`、`/wrist/right/depth/image_rect_raw` | `sensor_msgs/msg/Image` |

头部、手臂、腰部和双腿指令沿用 SDK [motocontrol 示例](../../../underlay/src/py_examples/py_examples/motocontrol.py)，
保存完整 `JointCommandArray`，包含各关节的名称、position、velocity、effort、stiffness 和 damping。
夹爪指令依据 SDK [hand_control 示例](../../../underlay/src/py_examples/py_examples/hand_control.py)及
[HandCommandArray](../../../underlay/src/aimdk_msgs/interface/robot/hal/msg/HandCommandArray.msg)，
保留左右夹爪类型及各自指令数组中的全部字段。所有指令来源的 QoS 为 best_effort／volatile、depth=100，
保留 `header.stamp`、`meas_stamp` 和 `sequence`，复用现有原始消息写入与收尾计数。

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

构建使用 `ament_cmake_python` 安装通用读回模块 `data_collection.readback`。同步本次读回改动后需重新构建并加载 overlay，使试次入口可以导入该模块。

统一入口启动腕部驱动时复用 `aimdk_functions` 已安装的 `config/wrist_cameras.json`，需要 `realsense2_camera`。按用途选一条构建命令。

```bash
# 采集 SDK 已发布的机器人数据
bash scripts/build_overlay.sh data_collection
# 独立 XR 录制并入包
bash scripts/build_overlay.sh remote_control data_collection
# 完整遥操试次
bash scripts/build_overlay.sh remote_control aimdk_functions data_collection
```

采集器负责启动腕部驱动时，将 `aimdk_functions` 一并构建。构建后按[常用采集流程](#常用采集流程)加载环境。

## 启动参数参考

数据根目录位于 `/home/agi/aimdk-fsc/data` 或其子目录，由项目目录挂载持久化并受同步 `/data/` 排除规则保护。远程清理任务仍按既有规则处理此目录。腕部设备序列号、命名空间及图像参数复用[腕部相机配置](../functions/README.md#左右腕部-d405-与单帧图像)，来源话题需与该配置一致。头部 RGBD、关节和 IMU 复用机器人已有发布者。

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

[手动起停与读回](#手动起停与读回)使用 `inspect_collection --verify-mcap`。工具遍历 MCAP、反序列化所有消息，核对话题、类型及逐话题写入数，检查图像尺寸、步长与数据长度，并报告源时间重复和回退。退出码 0 表示当前会话完整性规则通过，1 表示不完整，2 表示读取、结构或摘要不一致。仍需抽查图像内容、关节名称和实际数据意义。

不带 `--verify-mcap` 时仅用标准库检查摘要，新会话不读 MCAP，报告 `raw_mcap_verified=false`；可在本地使用。历史 schema_version=1 会话继续按原规则读取 `frames.jsonl`；历史原始 MCAP 也可在 ROS 环境使用上述读回选项。旧 `record_raw_mcap` 启动参数已移除，新录制始终保存 MCAP。

### XR 对账与时间配对

`record_trial.py verify` 一次扫描生成 `readback.json`。查看 `data_complete`（采集完整性）、`raw_mcap_verified`（MCAP 读回）和 `xr_readback`（XR 输入与对账）。完整试次从 `experiment.json` 读取目标设备、试次编号和 XR 日志引用，独立 XR 会话从同次 JSONL 获取设备和输入配置。退出码 0 表示完整性、MCAP 及对账通过，1 表示数据或 XR 质量不足，2 表示结构、读回或协议错误。

`xr_raw` 保存遥操进程发布的 `/remote_control/xr/raw`，类型为 `std_msgs/msg/String`。
JSON 信封 `remote_control.xr_raw.v1` 保留 SDK 原始 Base64 载荷、回调类型、设备、
`sdk_connection_id`、`callback_seq`、机器人侧系统／单调接收纳秒及退出排空标记。
遥操入口在解析及筛选前发布，使用 best-effort／volatile。原始 PICO 源时戳在载荷中，
MCAP 索引使用采集器的 ROS 接收时戳。`verify` 使用目标设备与实际输入配置核对 Tracking、
可解析双手帧、断连、序号缺口，并和 XR JSONL 按原始事件标识与内容对账。组合核验通过需要目标设备至少一帧可按实际输入协议解析的双手数据，以及同次 JSONL 对账一致。

对实际发过控制命令的会话，用同一 Humble 环境导出配对索引：

```bash
python3 scripts/align_trial.py "$SESSION" --stream arms --stream hands
```

`alignment.jsonl` 以控制日志中的实际发送目标为动作，按输入序号和回调标识关联 XR，保留
命令使用的反馈快照。其他观测按命令 ROS 时戳取同话题此前最近的 MCAP 接收样本，
输出从零开始的消息索引、纳秒差和缺样状态。用重复的 `--stream` 选择来源；提供
`--max-age-ms` 时，超龄样本记为缺样。会话摘要记录接收门开启、关闭附近的系统、ROS 和单调
时钟采样。该配对描述机器人侧接收时间，硬件同步与跨设备时钟偏差需另测。

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
2. 默认预设录制：确认双臂、夹爪反馈及胸部、骨盆 IMU 四个话题均有数据，原始关节名称、错误、IMU 及时间字段读回一致，新目录不生成 frames.jsonl。
3. 单独选择 `wrist_left_rgb`：确认仅左腕彩色流及内参写入，右腕不由本入口启动，独立相机会话能正常 STOP 和读回。
4. 使用 `manipulation`，再复用已启动驱动：确认头部 RGBD、双腕 RGB、反馈及指令逐话题有数据，内参存在；抽查图像与深度编码及标定内容。
5. 全量短录制：记录时长、MCAP 体积、实际接收频率、队列峰值、丢弃和 Jetson 负载；检查深度原始数值与容器压缩配置。
6. 使用 `whole_body_commands`：确认五路指令的话题与类型匹配；STOP 后读回 MCAP，核对关节名称、完整指令字段、夹爪类型与左右数组，以及源时间／测量时间／序号；逐话题 submitted=written、dropped=0。真机未发布的来源须显示缺流，不作为采集完成。
7. 重复 START、重复 STOP 及连续两次会话：确认幂等、收尾计数守恒、队列清空且消息不跨会话。已选来源不发布时如实报告无数据。
8. 示教与 XR 对账：录制一轮含双臂和夹爪动作的短示教，核对 Tracking、XR 对账、实际发布与观测配对，以及下载后的产物和标签。另录一轮只读断连，核对 MCAP 断连事件、进程退出原因及断连前的数据。

C++ 节点、MCAP 写入和 ROS 启动均由用户同步后在机器人容器构建、运行与验证。本地旧 JSONL 验证结果不代表本版多模态验收通过。
