# AimDK FSC

## 工作区分层

本地仍使用一个 Git 仓库，源码分为两个独立工作空间：

```text
underlay/
  src/aimdk_msgs、ruckig、examples、py_examples、robotop
  version、topics_and_services、run_docs.sh、docs/cn/
  extra/       # 强化学习参考工程及补充模型，忽略Git、不同步、不构建
overlay/
  src/functions、test、data_collection、locomotion_wrapper
scripts/       # 构建入口及本地远程清理工具
docs/          # 自研规划与实验记录
data/、log/、exports/
```

机器人端沿用 `/home/agi/aimdk` 作为固定厂商 underlay，
`/home/agi/aimdk-fsc` 作为可清空的自研 overlay，路径不增加上述本地目录层级。
本地 `underlay/` 仅保留 SDK 副本供查阅，不参与日常同步。
其中 `extra/` 保留 `COLCON_IGNORE`，当前厂商包和自研包均不以它作为构建或运行依赖。
固定 SDK 和本地副本应保持版本对应，升级 SDK 时重新核对并构建 underlay。
`locomotion_wrapper` 保留在本地 overlay 中，本轮不推送、不构建。

## 项目工作流程

### 1. 本地修改与代码同步

在本地工作区修改代码、完成必要的静态检查后，在 VS Code 中依次运行：

1. `机器人：预览同步`：使用 rsync 的 `--dry-run` 查看待同步与待删除内容。
2. `机器人：推送工作区`：确认预览后，实际推送代码。

当前目标为 `agi@10.0.1.41:/home/agi/aimdk-fsc/`，具体连接与参数以
[.vscode/tasks.json](.vscode/tasks.json) 为准。
两个任务使用 rsync `--relative` 的 `/./` 分界，将本地 `overlay/src/` 映射到
机器人 `aimdk-fsc/src/`，将根目录 `scripts/` 映射到 `aimdk-fsc/scripts/`。
只有这两个目录参与同步；厂商副本、`underlay/extra/`、本地研究文档和数据不会被推送。
沿用 `--delete`，只清理目标 `src/`、`scripts/` 中本地已不存在且未被排除规则保护的内容。
保留现有数据、日志、构建产物等排除规则，以及对 `src/locomotion_wrapper/` 的排除，
不使用 `--delete-excluded`。受保护的旧安装产物不会因推送自动清理。

### 2. 容器内构建与验证

机器人宿主机的项目目录为 `/home/agi/aimdk-fsc`，通过目录绑定挂载至容器内同一路径；
固定厂商目录 `/home/agi/aimdk` 和数据目录 `/home/agi/data-fsc` 也需在容器中同路径可见。
现有 `docker/run.sh` 已包含这三处挂载；已有容器是否具备挂载以实际创建配置为准。
在机器人 Humble 容器内构建、运行和测试依赖 ROS 2、SDK 或硬件的代码。

首次分层由用户停止自研程序、下载数据和日志，并清空机器人 `aimdk-fsc` 内容后再推送，
避免旧混合工作区的安装结果残留；保留 `/home/agi/aimdk`。
若固定 underlay 尚未构建，在未加载旧 FSC 安装环境的新容器终端执行一次：

```bash
cd /home/agi/aimdk
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select aimdk_msgs ruckig examples py_examples robotop
```

已有与当前 Humble 容器匹配的安装结果时可复用，无需重复构建。
上面的命令使用默认 `install/`；若已有安装目录名称不同，须核实并使用实际目录。
underlay 固定后，在另一个未加载旧 overlay 的容器终端构建自研包：

```bash
cd /home/agi/aimdk-fsc
bash scripts/build_overlay.sh
```

默认只构建 `aimdk_test`、`aimdk_functions`、`data_collection`，也可只构建指定包：

```bash
bash scripts/build_overlay.sh aimdk_functions
```

入口 [build_overlay.sh](scripts/build_overlay.sh) 默认加载
`/home/agi/aimdk/install/setup.bash`；通过环境变量 `AIMDK_UNDERLAY_INSTALL` 可指定其他安装目录。
它使用 `--base-paths src --packages-select`，不递归构建厂商包，不接受 locomotion 或其他包名。
采集器 SDK 版本从 underlay 安装目录同级 `version` 读取，可用 `AIMDK_SDK_VERSION_FILE`
指定真实版本文件；缺失时记录 `unknown`，不使用本地副本冒充机器人 SDK 版本。
脚本不自动安装依赖或构建 underlay，三个自研包可按实际依赖就绪情况分别构建。

每个运行终端加载厂商和自研安装环境，再执行所需命令：

```bash
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 pkg prefix aimdk_msgs
ros2 pkg prefix aimdk_functions
ros2 run aimdk_functions dual_arm_control --help
```

两项 prefix 应分别指向固定 underlay 和 FSC overlay 的安装目录。
自定义 underlay 安装路径也需用于运行终端。构建脚本在子进程中加载环境，不会修改调用终端。
工作区依赖链及 `setup`／`local_setup` 语义见
[colcon 官方说明](https://colcon.readthedocs.io/en/released/user/using-multiple-workspaces.html)。

本地双工作空间布局下，构建脚本可识别 `overlay/src/` 并定位工作区；
本地电脑仍仅用于编辑和静态检查，不执行依赖机器人环境的构建。

根据容器内构建和实机测试结果在本地继续修改，再次同步并验证。
进入容器的常用命令见下文；如容器名称或配置变更，先核对实际环境。

### 3. 下载数据并保存日志

1. 停止实验、自研节点和采集，等待录制文件关闭。
2. 在本地 VS Code 按需运行 `机器人：下载采集数据` 和 `机器人：下载日志`；清理前应完成所需数据及日志的下载。
3. 确认各任务正常结束，核对本地会话文件和日志，必要时读回数据。

`机器人：下载采集数据` 将 `agi@10.0.1.41:/home/agi/data-fsc/` 内的内容单向下载到
当前项目根目录的 `data/`，目录不存在时会创建。
`机器人：下载日志` 将 `agi@10.0.1.41:/home/agi/aimdk-fsc/log/` 内的全部内容
单向下载到本地 `log/`，包括双臂控制 JSONL 日志及该目录内的构建、调试和验证日志。
下载前应等待相关程序退出、日志写盘；日志任务要求机器人源目录已存在。

两个下载任务均不删除机器人源文件或本地独有文件；使用 `--checksum` 比较已有文件内容，
同路径内容不同时更新本地文件。每次采集应使用不同的会话目录名，避免覆盖历史数据。
两个任务独立运行，均不会自动触发清理。

### 4. 清理机器人工作内容

确认所需数据和日志已下载并核对，且没有自研节点继续使用相关目录后，
在本地项目根目录使用下文的清理命令：先预览，核对后再添加 `--execute` 实际清理。

脚本默认通过 SSH 在机器人宿主机清空 `/home/agi/data-fsc` 和 `/home/agi/aimdk-fsc/log`
的全部内容，包括隐藏文件，保留目录本身及权限。
添加 `--all` 时清理上述两处及整个 `/home/agi/aimdk-fsc`，包括源码、`build/`、`install/`
等内容；保留项目和数据根目录，项目内的 `log/` 目录也会被删除。
绑定挂载使清理结果同时反映到容器内，无需进入、停止或删除容器。

下载与清理独立运行；脚本不检查下载结果、不停止节点、不自动使用 sudo。
权限不足、连接中断或子挂载点等错误可能导致部分删除，须检查输出，不能视为清理成功。

### 5. 下次使用

保留现有容器及挂载配置时，默认清理后可继续使用现有代码和构建产物。
如果使用了 `--all`，下次重新同步后只需运行 `bash scripts/build_overlay.sh` 构建自研包，
再加载 underlay 和 overlay 环境运行。`/home/agi/aimdk` 不在清理范围内，厂商包无需重建；
通常无需重新构建镜像或重新创建容器。
全量清理会删除 `log/`，在它重新生成前重复运行现有清理脚本会因目录不存在而拒绝执行，
本轮保留这一既有检查行为。

连接地址变更时，同时更新 `.vscode/tasks.json` 和 `scripts/robot_cleanup.py`。

## 常用命令

### 进入开发容器

在机器人宿主机执行；当前记录的容器名为 `aimdk-dev-fsc`。

```bash
podman exec -it aimdk-dev-fsc bash
```

### 预览机器人目录清理

在本地项目根目录执行，仅列出待清理内容。

```bash
python3 scripts/robot_cleanup.py
```

### 执行机器人目录清理

停止实验和采集、下载并核对数据及日志后，在本地项目根目录执行。

```bash
python3 scripts/robot_cleanup.py --execute
```

如需同时清空整个项目目录，先预览全部范围，再执行：

```bash
python3 scripts/robot_cleanup.py --all
python3 scripts/robot_cleanup.py --all --execute
```
