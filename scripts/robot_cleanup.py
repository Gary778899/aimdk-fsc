#!/usr/bin/env python3
"""在本地通过 SSH 预览或清理机器人项目，保留 RealSense 工作空间。"""

import argparse
import subprocess
import sys


# 默认清理数据和 overlay 日志；全量清理保留 RealSense 工作空间，不跟随符号链接。
REMOTE_SCRIPT = r'''set -euo pipefail
roots=(
    /home/agi/aimdk-fsc/data
    /home/agi/aimdk-fsc/log
)
if [[ "$2" == '--all' ]]; then
    roots+=(/home/agi/aimdk-fsc)
fi

# 删除前先检查全部根目录；拒绝将根目录符号链接作为清理入口。
existing_roots=()
for root in "${roots[@]}"; do
    if [[ -L "$root" ]]; then
        printf '拒绝清理：目录为符号链接：%s\n' "$root" >&2
        exit 1
    elif [[ ! -e "$root" ]]; then
        printf '跳过：目录不存在：%s\n' "$root"
        continue
    elif [[ ! -d "$root" ]]; then
        printf '拒绝清理：路径不是目录：%s\n' "$root" >&2
        exit 1
    fi
    existing_roots+=("$root")
done

for root in "${existing_roots[@]}"; do
    printf '\n目录：%s\n' "$root"
    if [[ "$1" == '--execute' ]]; then
        # 包括隐藏文件，保留根目录，禁止跨文件系统遍历；任何删除错误均中止。
        find "$root" -xdev -depth -mindepth 1 \
            ! -path /home/agi/aimdk-fsc/realsense_ws \
            ! -path '/home/agi/aimdk-fsc/realsense_ws/*' -delete
        printf '已清空：%s\n' "$root"
    else
        printf '待删除的直接子项（子目录的全部内容也将删除）：\n'
        find "$root" -mindepth 1 -maxdepth 1 \
            ! -path /home/agi/aimdk-fsc/realsense_ws -printf '  %f\n'
    fi
done
'''


def main():
    """解析本地命令并调用 SSH；返回远程命令退出码，启动失败时返回 1。"""
    parser = argparse.ArgumentParser(
        description=(
            "通过 SSH 清空 /home/agi/aimdk-fsc 下的 data 和 log，"
            "添加 --all 同时清理项目源码及构建产物，保留 realsense_ws。默认仅预览。"
        ),
        epilog=(
            "执行前必须停止实验和采集，并完成数据下载与核对。"
            "本脚本不会下载数据、停止节点或自动使用 sudo。"
        ),
    )
    parser.add_argument(
        "--execute", action="store_true", help="实际删除所选目录内的全部内容"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="同时清理项目源码和构建产物，保留 realsense_ws 工作空间",
    )
    parser.add_argument(
        "--host", default="agi@10.0.1.41", help="SSH 目标，默认 agi@10.0.1.41（有线）"
    )
    args = parser.parse_args()
    mode = "--execute" if args.execute else "--preview"
    scope = "--all" if args.all else "--data-log"
    print(
        "实际清理：请确保已停止实验和采集，且数据已下载并核对。"
        if args.execute
        else "仅预览，不删除文件；添加 --execute 才会实际清理。",
        flush=True,
    )
    try:
        result = subprocess.run(
            ["ssh", args.host, "bash", "-s", "--", mode, scope],
            input=REMOTE_SCRIPT,
            text=True,
            check=False,
        )
    except OSError as exc:
        print(f"无法启动 SSH：{exc}", file=sys.stderr)
        return 1
    if result.returncode:
        print(
            "远程操作失败，可能已经清理部分内容；请检查输出，不要视为清理完成。",
            file=sys.stderr,
        )
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
