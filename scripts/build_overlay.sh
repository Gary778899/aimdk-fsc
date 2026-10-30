#!/usr/bin/env bash
# 在机器人 Humble 容器内构建自研包；不构建或修改固定厂商工作区。
set -eo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo '用法：bash scripts/build_overlay.sh [aimdk_test aimdk_functions data_collection]'
    echo '不传包名时构建上述三个包；locomotion_wrapper 不参与构建。'
    echo 'AIMDK_UNDERLAY_INSTALL：固定 underlay 安装目录，默认 /home/agi/aimdk/install。'
    echo 'AIMDK_SDK_VERSION_FILE：SDK 版本文件，默认 underlay 安装目录同级的 version。'
    exit 0
fi

packages=("$@")
if [[ ${#packages[@]} -eq 0 ]]; then
    packages=(aimdk_test aimdk_functions data_collection)
fi
for package in "${packages[@]}"; do
    case "$package" in
        aimdk_test|aimdk_functions|data_collection) ;;
        *) echo "不在本轮部署范围内的包：$package" >&2; exit 1 ;;
    esac
done

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# 本地仓库保留两层目录，机器人同步后直接使用 aimdk-fsc/src。
workspace="$project_root"
if [[ -d "$project_root/overlay/src" ]]; then
    workspace="$project_root/overlay"
fi
underlay_install="${AIMDK_UNDERLAY_INSTALL:-/home/agi/aimdk/install}"
underlay_install="$(realpath -m -- "$underlay_install")"
sdk_version_file="${AIMDK_SDK_VERSION_FILE:-$(dirname -- "$underlay_install")/version}"
sdk_version_file="$(realpath -m -- "$sdk_version_file")"

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$underlay_install/setup.bash" ]]; then
    echo "需要机器人 Humble 环境及已构建的 underlay：$underlay_install/setup.bash" >&2
    exit 1
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
    echo '请在未加载其他 ROS 发行版的机器人 Humble 容器终端构建。' >&2
    exit 1
fi
# ROS 生成的环境脚本可能读取未定义变量，因此不启用 nounset。
source /opt/ros/humble/setup.bash
source "$underlay_install/setup.bash"
cd -- "$workspace"
colcon build --base-paths src --packages-select "${packages[@]}" \
    --cmake-args "-DAIMDK_SDK_VERSION_FILE:FILEPATH=$sdk_version_file"
