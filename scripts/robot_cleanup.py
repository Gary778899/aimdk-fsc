#!/usr/bin/env python3
"""在本地通过 SSH 预览或清空机器人上的固定项目目录和数据目录。"""

import argparse
import subprocess
import sys


# 默认清理数据和日志；全量清理时增加项目目录，不跟随目录内的符号链接。
REMOTE_SCRIPT = r'''set -euo pipefail
roots=(/home/agi/data-fsc /home/agi/aimdk-fsc/log)
if [[ "$2" == '--all' ]]; then
    roots+=(/home/agi/aimdk-fsc)
fi

# 删除前先检查全部根目录；拒绝将根目录符号链接作为清理入口。
for root in "${roots[@]}"; do
    if [[ -L "$root" || ! -d "$root" ]]; then
        printf '拒绝清理：目录不存在或为符号链接：%s\n' "$root" >&2
        exit 1
    fi
done

for root in "${roots[@]}"; do
    printf '\n目录：%s\n' "$root"
    if [[ "$1" == '--execute' ]]; then
        # 包括隐藏文件，保留根目录，禁止跨文件系统遍历；任何删除错误均中止。
        find "$root" -xdev -depth -mindepth 1 -delete
        printf '已清空：%s\n' "$root"
    else
        printf '待删除的直接子项（子目录的全部内容也将删除）：\n'
        find "$root" -mindepth 1 -maxdepth 1 -printf '  %f\n'
    fi
done
'''


def main():
    """解析本地命令并调用 SSH；返回远程命令退出码，启动失败时返回 1。"""
    parser = argparse.ArgumentParser(
        description=(
            "通过 SSH 清空 /home/agi/data-fsc 和 /home/agi/aimdk-fsc/log，"
            "添加 --all 同时清空 /home/agi/aimdk-fsc。默认仅预览。"
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
        "--all", action="store_true", help="同时清空整个项目目录（含源码和构建产物）"
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
            ["ssh", "agi@10.0.1.41", "bash", "-s", "--", mode, scope],
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
