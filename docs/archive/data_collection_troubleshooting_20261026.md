# data_collection 历史排障与包名迁移记录

适用记录：2026-10-26起的X2机器人Ubuntu 22.04／ROS 2 Humble容器。
状态：2026-11-03从包README提取；保存当时的环境事实与处理过程。
当前操作入口见[采集包说明](../../overlay/src/data_collection/README.md)。下列修复命令仅适用于记录中的已核实条件，不能直接作为当前环境操作步骤。

## 依赖安装与只读挂载排障

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


## 包名迁移

采集包由 `data_collection_edge_agent` 重命名为 `data_collection`，源码目录为
`src/data_collection/`。主启动文件为 `data_collection.launch.py`，节点可执行文件与
默认节点名均为 `data_collection_node`；C++ 头文件前缀及命名空间改为
`data_collection`，节点类改为 `DataCollectionNode`。
`robot_data_collection.launch.py` 在当时作为新包内的启动别名，已于2026-11-03移除。
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
