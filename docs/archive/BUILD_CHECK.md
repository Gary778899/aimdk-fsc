# ROS 2 本机构建与检查记录

检查日期：2026-10-19。

后续新增的 WBC 部分迁移与验收见 `MIGRATION_PLAN.md`、`MIGRATION_REPORT.md`；其中采集节点仍等待 ZeroMQ 系统开发依赖。本文件原有五包检查记录不代表新增采集包已构建通过。

## 结论

主 SDK 工作区的 5 个包均已在本机编译、安装成功。后续补装 Python 依赖后，入口导入检查已全部通过；自带格式检查仍有历史失败，真机通信尚未验证。

### 最新复查：剩余 Python 依赖已安装

用户补装依赖后，在项目 `.venv` 内确认：`typer==0.24.1`、`textual==8.2.8`、`dateparser==1.4.3`、`ruckig==0.14.0` 均可导入。系统 `rclpy`、`cv_bridge` 也可导入。

重新运行 `scripts/check_ros2.py`，失败数为 0：38 个 Python 示例入口、2 个 robotop 入口均可导入；88 个消息、33 个服务的默认值序列化检查通过；37 个 C++ 程序无缺失动态库；63 个 Python 文件语法通过。`robotop --help`、日期解析检查也通过。这些结果不代表已验证真机通信或完整 TUI 交互。

最新结果见 `log/checks/venv-runtime-check.txt`、`log/checks/python-env.txt`、`log/checks/robotop-help.txt`。下文缺失依赖的记录均为此前安装阶段的历史状态。

### 后续：uv 虚拟环境安装

已按后续要求创建 `.venv`，使用 `/usr/bin/python3`（3.12）并启用 `--system-site-packages`，以访问系统 ROS 2 Python 包。最初缺少 `typer`、`textual`、`dateparser`、`ruckig` 四个模块，目前状态为：

| 模块 | 虚拟环境中的状态 |
| --- | --- |
| `typer` | 已用 uv 从本地缓存安装 0.24.1，并安装其缺少的依赖 |
| `ruckig` | 已用 uv 从项目自带源码构建、安装 0.14.0 Python 绑定 |
| `textual` | 未安装；没有本地缓存，PyPI DNS 解析失败 |
| `dateparser` | 未安装；没有本地缓存，PyPI DNS 解析失败 |

Typer 的必需依赖版本已核验；Ruckig 单轴轨迹在 318 步后到达目标位置。已在虚拟环境下重建 `py_examples`、`robotop`，安装后的命令入口使用 `.venv/bin/python`。复查中，38 个 Python 示例入口全部可导入，`robotop` TUI 仍因缺少 `textual` 无法导入。`rclpy`、`cv_bridge` 可正常导入。下面原始检查结果保留为安装前记录。

每次使用：

```bash
source /opt/ros/kilted/setup.bash
source install/local_setup.bash
source .venv/bin/activate
```

网络恢复后继续安装剩余依赖：

```bash
UV_CACHE_DIR=/tmp/aimdk-uv-cache uv pip install --python .venv/bin/python textual dateparser
python scripts/check_ros2.py
```

补充依赖清单见 `requirements-runtime.txt`。此清单不包含继承自系统的 ROS 2 依赖，也不是完整锁文件。新检查日志为 `log/checks/venv-runtime-check.txt` 和 `log/checks/venv-python-build.txt`。

## 项目与环境

| 路径 / 包 | 用途 |
| --- | --- |
| `src/aimdk_msgs` | 88 个消息、33 个服务的接口定义及类型支持 |
| `src/ruckig` | C++ 实时轨迹生成库；目前 CMake 不构建 Python 绑定 |
| `src/examples` | C++ 示例，安装了 37 个 ELF 可执行文件 |
| `src/py_examples` | Python 示例，38 个命令入口 |
| `src/robotop` | 终端诊断界面、Orin 监控节点 |
| `docs` | SDK 本地 HTML 文档 |
| `extra` | 独立的强化学习、部署、仿真相关工程，未纳入本次 SDK 构建 |

本机为 Ubuntu 24.04、x86_64、ROS 2 Kilted、Python 3.12。随项目提供的文档要求 Ubuntu 22.04、ROS 2 Humble、C++17，并明确支持上位机通过 ROS 2 连接机器人。

ARM64 预编译库只适用于对应架构，但 SDK 源码可在 x86_64 笔记本重新构建。ROS 2、Python、系统库版本也会影响二进制兼容性。本次 Kilted 构建结果不代表已验证与机器人 Humble 环境互通。

## 本次修复

- 移除强制设置的 ARM64 OpenCV 路径，让 CMake 自动发现本机 OpenCV。
- 为 `cv_bridge` 的 `.h` / `.hpp` 头文件提供兼容入口。
- 修复 `play_audio`、`play_tts` 的异步请求结果向 `shared_future` 转换失败。
- 在 C++ 示例的 `package.xml` 中补充 `rclcpp`、`sensor_msgs`、`cv_bridge`。
- 预编译缓存更新脚本仅在安装目录存在 `local` 时复制该目录，适配本机 Python 安装布局。
- 添加 `scripts/check_ros2.py`，可检查接口序列化、入口导入、动态库与 Python 语法，不调用机器人服务。

## 复现构建

在项目根目录执行：

```bash
source /opt/ros/kilted/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build \
  --base-paths src --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
```

必须限定 `--base-paths src`：`extra` 中包含同名接口包与其他独立工程，不能把整个目录当作同一个工作区递归构建。未使用 `--symlink-install`，符合 `robotop/README.md` 的安装要求。

构建输出位于 `build/`、`install/`、`log/`。项目原有安装逻辑还会自动生成 `src/aimdk_msgs/prebuilt_x86_64/`；该缓存对应本次系统、ROS 2 和 Python 环境，不应直接用于其他 ROS 2 发行版。

## 检查结果

| 检查 | 结果 |
| --- | --- |
| `colcon build` | 5 个包成功 |
| 接口原生类型支持、默认值序列化往返 | 88 个消息、33 个服务通过 |
| C++ 动态链接依赖 | 37 个可执行文件，无缺失库 |
| Python 语法 | 63 个文件通过 |
| Python 示例入口 | 37/38 可导入；`motocontrol` 缺少 Python `ruckig` |
| robotop 入口 | 监控节点入口可导入；TUI 缺少 `typer`，另确认缺少 `textual`、`dateparser` |
| ROS 节点初始化 | 节点对象可创建，但 DDS 报套接字权限错误，不能据此判断通信可用 |
| 项目自带测试 | 未全部通过，见下文 |

本次接口检查使用默认字段值，不覆盖所有业务数据组合。未进行真机、传感器、运动控制或强化学习仿真验证。

### 自带检查与遗留问题

执行了 `colcon test --base-paths src --executor sequential --return-code-on-test-failure` 和 `colcon test-result --all`。

- `aimdk_msgs`：flake8、CMake 格式检查失败；uncrustify 超时。检查会递归扫描随 SDK 提供的 ARM64 和自动生成的 x86_64 预编译目录，包含大量生成代码。
- `examples`：Python/C++ 格式检查存在失败。
- XML 校验无法下载 `http://download.ros.org/schema/package_format3.xsd`；该失败不能直接解释为 XML 内容错误。
- cppcheck 用例被跳过；Ruckig 未配置测试。
- 两个 Python 包在默认 colcon 测试流程中未发现测试，以返回码 5 退出。另在 `src/py_examples` 目录直接执行了 pytest，其格式检查结果保存在日志中；`robotop` 未提供测试。
- 初次完整编译中，GCC 13 对 Ruckig 小数组排序发出 `-Warray-bounds` 警告，另有未使用参数警告；尚未证明该数组警告是否涉及实际越界，未修改第三方算法。
- Python 示例安装有 `py_examples.data` 包发现警告，但数据文件与命令已安装。
- `rosdep` 尚未初始化，不能完成依赖清单的自动核验。

本次未批量改写 SDK / 生成代码的格式，以免将构建适配扩大为无关修改。

### 缺失的运行依赖

尝试在项目内临时虚拟环境安装 `typer textual dateparser ruckig`，因 PyPI 域名解析失败而未能安装；空虚拟环境已清理，系统 Python 未修改。

网络恢复后可创建能访问系统 ROS 2 Python 包的独立环境，并安装上述依赖。安装后需要重新执行入口检查；安装依赖的结果尚未在本次环境验证。项目中的 C++ Ruckig 库不等同于 `import ruckig` 所需的 Python 模块。

## 复现检查与日志

加载 ROS 2 和工作区后执行：

```bash
python3 scripts/check_ros2.py
colcon test --base-paths src --executor sequential --return-code-on-test-failure
colcon test-result --all
(cd src/py_examples && python3 -m pytest -q test)
```

`scripts/check_ros2.py` 在依赖缺失时返回非零值，这是当前预期结果。

- `log/checks/build.txt`：最终成功构建日志。
- `log/checks/initial-build.txt`：初次构建及编译警告。
- `log/checks/runtime-check.txt`：接口、动态库及 Python 入口检查。
- `log/checks/node-check.txt`：节点创建与 DDS 权限错误。
- `log/checks/colcon-test.txt`、`test-results.txt`：自带测试及汇总。
- `log/checks/py-examples-pytest.txt`、`py-examples-pytest.xml`：Python 格式检查。
