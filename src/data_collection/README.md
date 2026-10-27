# data_collection 使用说明

当前包版本：`0.2.0`。目标环境为机器人开发板的 Ubuntu 22.04／ROS 2 Humble 容器。
2026-10-26 用户已在机器人容器完成一次录制；下载数据本地读回共 12,869 帧，实际平均输出 50.002 Hz，正常收尾、无统一帧写盘丢弃，双臂字段有效，但夹爪因名称映射不符全程缺失。随后用户实际反馈确认名称为 `left_claw_joint`／`right_claw_joint`，已据此修正默认映射并补充停止日志；这两项修改仅完成本地静态检查，仍待同步后在容器复验。两路测量时间戳为零、手部源序号为零的问题仍保留质量标记，不能据此认定完整数据验收通过。

## 功能与数据范围

独立订阅机器人已有反馈，通过 JSON 映射生成默认 **50 Hz（20 ms 一帧）** 的双臂与夹爪状态，保存为 JSONL；可额外保存原始 MCAP。不依赖 `locomotion_wrapper`、测试包或控制脚本，不发送运动命令或切换机器人模式。

| 来源 | 默认话题／消息类型 | 采集内容 |
| --- | --- | --- |
| 双臂 | `/aima/hal/joint/arm/state`／`aimdk_msgs/msg/JointStateArray` | 左右各 7 关节的位置、速度、原始 effort、错误码及整组状态 |
| 双夹爪 | `/aima/hal/joint/hand/state`／`aimdk_msgs/msg/HandStateArray` | 左右开度、原始速度／effort、运动状态、故障码及手型 |

每侧关节按 `shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll` 输出，输入按名称匹配。关节位置为 rad、速度为 rad/s；夹爪开度为 0～1，0 闭合、1 全开。`effort_raw` 保留 SDK 原值，未核实前不作为真实关节力矩或夹持力使用。

采样取各来源最近已收到的状态，不插值；保留各流源时间、接收时间、源序号、frame_id、接收年龄及复用标记。缺失或非有限字段用 `null` 和问题标记表示，故障反馈仍保留。50 Hz 是统一帧的目标生成频率，不保证每帧都有新的来源数据，也不代表硬件同步。

## 同步与构建

本地先执行 VS Code 的“机器人：预览同步”，确认后再执行“机器人：推送工作区”。两项任务均已纳入本包的源码、JSON 和启动文件；其他排除规则保持原样，`*.md` 仍不推送。

以下命令在**机器人开发容器的工作区根目录**执行。示例使用 colcon 默认的 `build/`、`install/`、`log/`；已有其他构建布局时使用实际对应路径。

### 构建前准备依赖

依赖声明见 [package.xml](package.xml)。`colcon build` 和 `source install/setup.bash` 均不会安装系统依赖。下面的清单以已有 SDK、ROS 2 Humble C++ 开发环境及可用的编译工具链为基础，包含本包直接使用的系统包及 rosbag2／MCAP 插件加载链需要的系统开发包：

| Ubuntu 22.04 系统包 | 用途／引入位置 |
| --- | --- |
| `pkg-config` | CMake 查找 ZeroMQ |
| `libzmq3-dev` | ZeroMQ C 库、`libzmq.pc` 及 C++ 接口 `zmq.hpp` |
| `nlohmann-json3-dev` | JSON 映射解析、JSONL 帧及会话元数据；对应 rosdep 键 `nlohmann-json-dev` |
| `python3-zmq` | Python 控制客户端 |
| `libyaml-cpp-dev` | `rosbag2_storage → yaml_cpp_vendor → yaml-cpp` |
| `libzstd-dev` | `mcap_vendor → zstd_vendor → zstd`；同时提供头文件与链接库，仅装 `zstd` 命令行工具不足 |
| `libtinyxml2-dev` | `rosbag2_cpp／rosbag2_storage → pluginlib → tinyxml2_vendor` |
| `libconsole-bridge-dev` | `pluginlib → class_loader → console_bridge_vendor` |

Ubuntu 22.04（Jammy）的 `libzmq3-dev` 已包含 `/usr/include/zmq.hpp`、`zmq_addon.hpp` 和架构对应的 `pkgconfig/libzmq.pc`，见 [Ubuntu 官方 arm64 文件清单](https://packages.ubuntu.com/jammy/arm64/libzmq3-dev/filelist)。本环境不单独安装 `cppzmq-dev`；此前将其他发行版的拆包方式套用于 Jammy 的说明和安装命令已修正。apt 若因某个包名无法定位而退出，该次命令不会完成其他包的安装，需移除错误包名后重新执行。

先核实容器的安装目录和包管理状态。2026-10-26 用户反馈确认：`/opt/ros` 是只读 overlay 挂载；apt 尝试安装的 7 个 ROS 包均为 `install reinstreq half-installed`，缺少文件清单和校验文件。用户随后已单独配置 `libzstd-dev:arm64`，确认状态为 `install ok installed`；CMake 已推进到第 18 行的 ZeroMQ `libzmq` 检查。此前 CMake 已能读取相应 ROS 配置文件，说明现有 ROS 文件不能单凭 apt 数据库中的“未安装”判定为缺失。检查命令：

```bash
findmnt -T /opt/ros/humble -o TARGET,SOURCE,FSTYPE,OPTIONS
findmnt -T /opt/ros/humble/include/mcap_vendor/mcap/crc32.hpp \
  -o TARGET,SOURCE,FSTYPE,OPTIONS
dpkg --audit
```

若先前的 ROS 包安装已失败，先根据 `dpkg --audit` 和挂载信息确定恢复方式，不直接运行 `apt-get -f install`、强制删除 ROS 包或将挂载改为可写。SDK 原有 ROS 文件与 dpkg 数据库的归属关系核实前，这些操作可能重复触发只读写入失败或处理 SDK 提供的文件。

对于上述已确认的 `libzstd-dev:arm64` 解包状态，可仅完成该系统开发包的配置，无需重新安装 ROS 包：

```bash
dpkg --configure libzstd-dev:arm64
dpkg-query -W -f='${binary:Package}\t${Status}\t${Version}\n' libzstd-dev
```

配置成功不会自动修复 7 个 ROS 包的 dpkg 状态。用户进一步检查 `/var/lib/dpkg/info/`，对应 7 个失败 ROS 包均未找到信息文件，即没有留下文件清单或维护脚本。在这些已核实条件下，可以先备份 dpkg 数据库，再定向移除本次失败安装的记录。以下命令仅适用于这组状态；后续环境变化时应重新核对，不能用于正常安装的 ROS 包：

```bash
(
  set -e
  ros_dpkg_backup=$(mktemp -d /root/dpkg-before-ros-cleanup.XXXXXX)
  cp -a /var/lib/dpkg "$ros_dpkg_backup/"
  printf 'dpkg backup: %s\n' "$ros_dpkg_backup"
  dpkg --no-triggers --remove --force-remove-reinstreq \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-storage \
    ros-humble-mcap-vendor \
    ros-humble-shared-queues-vendor \
    ros-humble-yaml-cpp-vendor \
    ros-humble-zstd-vendor
  dpkg --audit
)
```

`--force-remove-reinstreq` 允许处理标记为需要重装的失败包；这里的依据是这些包在本次新安装前未登记，安装因只读挂载失败，而且当前不存在对应 dpkg 信息文件。命令保留依赖检查，并禁止本步处理其他包的待执行触发器。若移除失败或审计仍报告问题，先根据输出处理，不继续 apt 安装。2026-10-26 用户已完成上述备份和定向移除，随后 `dpkg --audit` 无输出，确认此次失败记录已清理；容器构建仍未通过。

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

本包直接依赖的 ROS 包为 `ament_cmake`、`rclcpp`、`rmw`、`builtin_interfaces`、SDK 的 `aimdk_msgs`、`rosbag2_cpp`、`rosbag2_storage`、`mcap_vendor`，以及运行时的 `launch`、`launch_ros`、`ament_index_python`、`rosbag2_storage_mcap`。标准 apt 安装会递归安装 ROS 包声明的依赖；仅有 `/opt/ros/humble/share/<包名>` 或 CMake 配置文件，不能证明对应系统开发包齐全。

当前机器人容器先复用 SDK 已提供的 ROS 文件，补齐其需要的 Ubuntu 系统开发包；`aimdk_msgs` 继续使用 SDK 工作区提供的版本。若确认某个 ROS 包实际缺失，应先核对 SDK 容器的挂载、版本及厂家支持的安装方式，再决定如何补齐。仅在 ROS 安装路径可写、由 apt 管理且与 SDK 兼容的环境中，才适用标准 `ros-humble-*` apt 安装流程。

当前 CMake 无条件查找 rosbag2／MCAP，因此 `record_raw_mcap=false` 不会免除构建依赖。Humble 官方 `mcap_vendor` 将 LZ4 源码编入自身库，上述链路不额外要求系统 `liblz4-dev`；SQLite 存储插件也不是当前 MCAP 录制所需依赖。依据：[Humble MCAP 构建定义](https://github.com/ros2/rosbag2/blob/humble/mcap_vendor/CMakeLists.txt)、[pluginlib 依赖](https://github.com/ros/pluginlib/blob/humble/pluginlib/package.xml)、[class_loader 依赖](https://github.com/ros/class_loader/blob/humble/package.xml)、[rosdep 系统包映射](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml)。实际容器版本及缺包情况需在容器内核查，以上清单尚未经机器人容器构建验收。

### 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to data_collection
source install/setup.bash
```

CMake 只检查依赖，不自动下载或安装。沿用 SDK 的消息与预编译资源，不清理原有安装产物。

## 启动与录制

先选择容器内**已确认持久化、可写、位于同步工作区之外**的数据根目录。下面的 `/ABSOLUTE/PERSISTENT/DATA_DIR` 是占位符，必须替换为实际绝对路径。当前 rsync 使用 `--delete`，工作区内的 `rosbags/` 未受同步排除保护。

```bash
ros2 launch data_collection data_collection.launch.py \
  data_root:=/home/agi/data-fsc
```

节点启动后等待控制指令，不自动录制。在同一容器的另一个终端加载相同 ROS／工作区环境，再执行：

```bash
ros2 run data_collection collection_control START
ros2 run data_collection collection_control STATUS
# 完成采集后停止并等待文件关闭。
ros2 run data_collection collection_control STOP
```

控制客户端使用 `#!/usr/bin/env python3`，因此实际解释器由该终端的 `PATH` 决定。若提示无法导入 `zmq`，在同一容器内安装 `python3-zmq`，再检查 `python3 -c 'import sys, zmq; print(sys.executable); print(zmq.__version__)'`。若系统包已安装但仍失败，可用 `/usr/bin/python3` 执行同一检查，区分系统包缺失与虚拟环境／其他 Python 解释器的路径差异。新版客户端会显示解释器路径及原始导入异常；此类导入失败发生在发送请求前，START 尚未发送，无需因此重启已就绪的采集节点。

`START`／`STOP` 正常返回 `ACK`，失败返回 `ERROR ...`；录制中重复 START、正常停止后重复 STOP 均幂等。START 的 ACK 仅表示存储已就绪，仍需查看 STATUS 中的各流统计，确认实际收到数据。STATUS 返回 JSON，含 `session_dir`、运行状态、写入／丢弃计数及来源质量信息。

STOP 成功排空队列、关闭文件并写入摘要后，节点终端输出 `Recording stopped: ...; frames=...; data_complete=...`；重复 STOP 输出 `Recording already stopped`。收尾失败仍返回 ERROR 并记录异常，不输出成功停止日志。STOP 结束当前录制，节点继续运行，等待 STATUS 或下一次 START；正常有数据会话停止后可核对 STATUS 中 `sampling_enabled=false`、`closed=true`、`complete=true`、`queue_pending=0`。`data_complete=false` 仍可能来自字段或来源时间质量问题，不等同于停止失败。

客户端默认连接 `tcp://127.0.0.1:5555`，等待超时为 5000 ms，可通过 `--endpoint`、`--timeout-ms` 修改。STOP 超时不代表文件已经关闭，应继续查询 STATUS。

| 启动参数 | 默认值／用途 |
| --- | --- |
| `data_root` | 必填，持久化数据根目录的绝对路径 |
| `mapping_file` | 安装后的 `config/x2_upper_body.json` |
| `record_raw_mcap` | `false`；设为 `true` 时额外按来源频率录制完整 SDK 原始消息 |
| `queue_capacity` | `512`；异步写盘队列的记录条目容量，满时丢弃新记录并计数 |
| `zmq_bind_address`／`zmq_port` | `127.0.0.1`／`5555` |
| `robot_model`／`firmware` | 均为 `unknown`，可填写实际机型与固件版本，写入会话元数据 |

旧入口 `robot_data_collection.launch.py` 是当前独立入口的别名，参数相同。旧 `bag_root_dir`、`joint_state_topic`、`imu_topic` 参数已不适用。

## 文件与读回

每次新会话创建独立的 `data_<YYYYMMDD>_<HHMMSS>_<九位纳秒>_UTCp0800_<PID>_<序号>/` 目录，例如：

```text
data_20261026_202654_020248086_UTCp0800_43309_0/
```

日期与时分秒固定使用北京时间（UTC+08:00），不依赖容器默认时区；时间取自 START 初始化会话时的系统时钟，不代表第一帧传感器时间。保留纳秒、进程 PID、从零开始的进程内递增序号及目录创建冲突重试，避免会话重名。`session.json` 中的 `started_system_time_ns` 等整数纳秒字段保持不变。

该命名规则仅用于更新后的新会话；历史 `collection_<Unix纳秒>_<PID>_<序号>/` 目录无需重命名，仍可用相同工具读回。此调整仅完成本地静态检查，待同步后在机器人容器内构建与录制验证。

| 文件／目录 | 内容 |
| --- | --- |
| `frames.jsonl` | 每行一帧，包含 `arms`、`grippers`、`streams` 和整数纳秒时间 |
| `session.json` | 映射配置快照、版本／环境元数据、计数、关闭结果和质量摘要 |
| `raw/` | 仅启用原始录制时生成，保存 rosbag2／MCAP |

STOP 后，将 STATUS 返回的实际 `session_dir` 代入；下方 `/ABSOLUTE/SESSION_DIR` 也是占位符：

```bash
ros2 run data_collection inspect_collection /ABSOLUTE/SESSION_DIR
```

工具输出 JSON 报告，核对帧结构、关节顺序、时间推进与会话计数，并统计实际输出频率、采样间隔、来源复用、缺失、无效数据和故障。退出码：`0` 表示按当前规则数据完整，`1` 表示质量不完整或未正常收尾，`2` 表示文件损坏、摘要矛盾或读取失败。

`complete` 表示会话已正常关闭且没有存储错误，`data_complete` 还检查无数据、缺流、无效字段、已观察异常及记录丢弃。仍应结合实际频率、复用比例与接收年龄判断可用性，不能据此认定有 50 Hz 的独立新样本。未收尾会话会报告实际可读数据及摘要差异；当前工具**不验证原始 MCAP 内容**。本版本不提供 CSV 导出或动作回放。

## 包名迁移

采集包由 `data_collection_edge_agent` 重命名为 `data_collection`，源码目录为
`src/data_collection/`。主启动文件为 `data_collection.launch.py`，节点可执行文件与
默认节点名均为 `data_collection_node`；C++ 头文件前缀及命名空间改为
`data_collection`，节点类改为 `DataCollectionNode`。
`robot_data_collection.launch.py` 继续作为新包内的启动别名。
旧包名和 `data_collection_edge_agent.launch.py` 不再提供兼容入口；外部脚本中的
`ros2 run`、`ros2 launch`、包资源查询及节点名引用须同步更新。
本次只调整名称，录制参数、ZMQ 协议、映射与数据格式保持不变，已有采集数据无需迁移。

同步后在机器人容器内完成以下步骤：

1. 停止仍在运行的旧采集节点，确认录制已正常收尾。
2. 核对容器实际构建／安装目录、overlay 加载顺序和旧包来源。现有 rsync 排除规则保护
   构建及安装产物，源码改名不会自动移除旧包；确认后定向清理旧包产物。
   若为合并安装布局，不能把整个共享安装目录当作旧包目录删除。
3. 在未加载旧工作区 overlay 的终端加载 SDK／Humble 环境，按本文命令构建新包，
   再加载实际安装目录的 `setup.bash`。若使用自定义构建／安装目录，保留相应参数。
4. 用 `ros2 pkg prefix data_collection`、`ros2 pkg executables data_collection` 核对新包来源
   及三个可执行入口；用 `ros2 pkg prefix data_collection_edge_agent` 排查残留旧包。
5. 使用新入口录制、停止并读回一次数据，确认映射加载和文件收尾；若使用启动别名，
   同时验证该入口。改名前的验证结果不能替代改名后的容器构建与运行验证。

此次重命名仅完成本地静态检查，尚未完成机器人容器内的构建与运行验证。

## 修改映射与迁移平台

以 [config/x2_upper_body.json](config/x2_upper_body.json) 为模板，主要配置如下：

- `sample_rate_hz`：统一帧目标频率，默认 50。
- `streams`：来源标识、话题、消息类型、QoS、时间／序号字段路径及可选 `frame_id_path`、`state_path`。
- `arms.left/right`：来源数组、名称字段、7 个关节的来源名称与字段映射；`joint_fields` 可按统一关节名覆盖单个关节的完整字段规则。
- `grippers.left/right`：来源数组、夹爪名称、手型路径与期望值、开度和状态字段映射；当前 X2 实机反馈核实为左侧 `left_claw_joint`、右侧 `right_claw_joint`，期望手型为 `CLAW=2`。其他机型／固件仍应核对实际反馈名称。映射在节点启动时加载，修改后须重新启动节点；旧会话中的 null 夹爪字段不会被补回。

数值转换为 `输出 = 原值 × scale + offset`，可配置位置／速度的单位、方向与零位；`*_raw` 和整数状态字段保持原值，不允许缩放或偏置。配置启动时校验并冻结，修改后需重启节点。

通过 `mapping_file:=/ABSOLUTE/PATH/platform.json` 选择自定义配置。当前输入适配器仅支持表中的两种 SDK 消息；相同消息结构主要修改 JSON，其他消息结构或通信协议需要补充输入适配器，不能仅更改消息类型字符串。
