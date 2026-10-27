#!/bin/bash
# 复用开发容器；首次创建时挂载宿主机的 ROS、CUDA、cuDNN/TRT 和 aima。
set -euo pipefail

IMAGE="${IMAGE:-localhost/aimdk-dev-fsc:deps-20261027}"
NAME="${NAME:-aimdk-dev-fsc}"

# 已有容器沿用创建时的镜像、挂载和环境变量，保留其中安装的依赖。
# 修改下方创建参数不会更新已有容器；需要应用时应另行明确重建。
if podman container exists "$NAME"; then
  if [ "$(podman inspect --format '{{.State.Running}}' "$NAME")" = "true" ]; then
    echo "$NAME is already running"
  else
    podman start "$NAME"
  fi
  exit 0
else
  # exists 返回 1 表示不存在；其他错误直接退出，避免误入创建流程。
  container_exists_status=$?
  if [ "$container_exists_status" -ne 1 ]; then
    exit "$container_exists_status"
  fi
fi

# 仅为宿主机存在的来源添加挂载。
MOUNTS=()
m() {
  # 用途：添加可选挂载；参数为来源、目标、选项（默认 ro），来源不存在时提示并跳过。
  local src="$1" dst="$2" opts="${3:-ro}"
  if [ -e "$src" ]; then
    MOUNTS+=("-v" "$src:$dst:$opts")
  else
    echo "  mount $src NOT present, skipping" >&2
  fi
}

# Host GIDs to add to the container (groups owning device nodes).
HOST_GIDS=(
  20    # dialout - /dev/ttyUSB*
  29    # audio   - /dev/snd
  44    # video   - /dev/nvmap, /dev/nvhost-* (required by CUDA)
  106   # input   - /dev/input/event*
  109   # render  - /dev/dri/renderD*
  995   # debug   - /dev/nvhost-dbg-*
  1000  # run     - /agibot/data/var/MapManagerModule
)

# Translate host GIDs to container-side GIDs via the rootless ID mapping.
declare -a GROUP_ADDS
MAP_RAW=$(podman info --format '{{.Host.IDMappings.GIDMap}}')
for HG in "${HOST_GIDS[@]}"; do
  CG=$(echo "$MAP_RAW" | grep -oE '\{[0-9]+ '"$HG"' [0-9]+\}' | head -1 \
       | grep -oE '^\{[0-9]+' | tr -d '{')
  if [ -z "$CG" ]; then
    echo "GID $HG not mapped in /etc/subgid, skipping" >&2
  else
    GROUP_ADDS+=("--group-add" "$CG")
    echo "  $HG (host) -> $CG (container)" >&2
  fi
done

# Enumerate userspace CUDA shared libraries to bind-mount (cuDNN, TensorRT, cuBLAS, cuFFT, ...).
for f in /usr/lib/aarch64-linux-gnu/libcudnn*.so* \
         /usr/lib/aarch64-linux-gnu/libnvinfer*.so* \
         /usr/lib/aarch64-linux-gnu/libnvonnxparser*.so* \
         /usr/lib/aarch64-linux-gnu/libnvparsers*.so* \
         /usr/lib/aarch64-linux-gnu/libnvcaffe_parser*.so* \
         /usr/lib/aarch64-linux-gnu/libcublas*.so* \
         /usr/lib/aarch64-linux-gnu/libcufft*.so* \
         /usr/lib/aarch64-linux-gnu/libcurand*.so* \
         /usr/lib/aarch64-linux-gnu/libcusolver*.so* \
         /usr/lib/aarch64-linux-gnu/libcusparse*.so* \
         /usr/lib/aarch64-linux-gnu/libcupti*.so* \
         /usr/lib/aarch64-linux-gnu/libnpp*.so* \
         /usr/lib/aarch64-linux-gnu/libnvjpeg*.so* \
         /usr/lib/aarch64-linux-gnu/libnvToolsExt*.so* \
         /usr/lib/aarch64-linux-gnu/libnvrtc*.so* ; do
  m "$f" "$f"
done
echo "  CUDA libs to bind-mount: $(ls /usr/lib/aarch64-linux-gnu/libcuda*.so* 2>/dev/null | wc -l) files" >&2

# ncurses dev files (headers, link-time .so symlinks, pkg-config) for the keyboard example.
for f in /usr/include/curses.h /usr/include/ncurses*.h /usr/include/term.h \
         /usr/include/termcap.h /usr/include/unctrl.h /usr/include/eti.h \
         /usr/lib/aarch64-linux-gnu/libncurses*.so* /usr/lib/aarch64-linux-gnu/libtinfo*.so* \
         /usr/lib/aarch64-linux-gnu/pkgconfig/ncurses*.pc /usr/lib/aarch64-linux-gnu/pkgconfig/tinfo*.pc; do
  m "$f" "$f"
done

# Python include dirs (numpy, ...) that the host /opt/ros CMake configs reference by absolute path.
while read -r d; do
  m "$d" "$d"
done < <(grep -rhoE '/[A-Za-z0-9_./+-]*(dist|site)-packages/[A-Za-z0-9_./+-]*include' \
           --include='*.cmake' /opt/ros/humble/share 2>/dev/null | sort -u)

# Extract AGIBOT_* env vars from the firmware-managed bashrc into an env-file.
AGIBOT_BASHRC="/home/agi/.aima/env/bashrc"
ENV_FILE=$(mktemp /tmp/aimdk-env.XXXXXX)
trap 'rm -f "$ENV_FILE"' EXIT
if [ -r "$AGIBOT_BASHRC" ]; then
  sed -nE "s/^export (AGIBOT_[A-Z_]+)='(.*)'\$/\1=\2/p" "$AGIBOT_BASHRC" > "$ENV_FILE"
  echo "  AGIBOT env vars: $(wc -l < "$ENV_FILE") loaded from $AGIBOT_BASHRC" >&2
else
  echo "  $AGIBOT_BASHRC not readable, skipping AGIBOT_* env" >&2
fi

# If the DDS config exported by the firmware exists, mount it at the container path
# referenced by FASTRTPS_DEFAULT_PROFILES_FILE (shared discovery with the robot).
m /home/agi/.aima/env/ros_dds_configuration.xml /etc/ros_dds_configuration.xml

# ---- single fixed mount: the SDK dir is always present and must be rw ----
m /home/agi/aimdk /home/agi/aimdk rw
m /home/agi/aimdk-fsc /home/agi/aimdk-fsc rw
m /home/agi/data-fsc /home/agi/data-fsc rw

echo "starting $NAME from $IMAGE..."
podman run -d --name "$NAME" \
  --ipc=host --network=host --pid=host \
  --device nvidia.com/gpu=all \
  "${GROUP_ADDS[@]}" \
  -e HOME=/home/agi \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros_dds_configuration.xml \
  -e PATH=/opt/ros/humble/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  -e LD_LIBRARY_PATH=/usr/local/lib:/usr/local/cuda/lib64:/opt/ros/humble/lib:/host-libs/aarch64-linux-gnu \
  --env-file "$ENV_FILE" \
  -v /etc/machine-id:/etc/machine-id:ro \
  -v /usr/lib/aarch64-linux-gnu:/host-libs/aarch64-linux-gnu:ro \
  -v /opt/ros:/opt/ros:ro \
  -v /usr/local/bin/aima:/usr/local/bin/aima:ro \
  -v /dev:/dev:rslave \
  -v /run/dbus:/run/dbus:rslave \
  "${MOUNTS[@]}" \
  "$IMAGE"
