#!/usr/bin/env bash
# 下载机器人采集数据。第三个参数为 all 或 experiments 下的任务/会话相对路径。
set -eu

destination="$1"
robot_host="$2"
trial="${3:-all}"
remote="/home/agi/aimdk-fsc/data"

if [[ "$trial" != all ]]; then
    if [[ ! "$trial" =~ ^[a-zA-Z0-9_-]+/[0-9]{8}_[0-9]{6}(_[0-9]+)?$ ]]; then
        echo '请填 all 或 任务/YYYYMMDD_HHMMSS[_序号]。' >&2
        exit 2
    fi
    remote="$remote/experiments/$trial"
    destination="$destination/experiments/$trial"
fi

status=0
ssh -- "$robot_host" "if [ -d '$remote' ]; then exit 0; elif [ ! -e '$remote' ]; then exit 3; else exit 1; fi" || status=$?
if [[ "$status" -eq 3 ]]; then
    if [[ "$trial" == all ]]; then
        echo '无数据可下载，机器人 data 目录不存在。'
        exit 0
    fi
    echo "机器人会话目录不存在：$trial" >&2
    exit 3
elif [[ "$status" -ne 0 ]]; then
    exit "$status"
fi

if [[ "$trial" != all ]]; then
    ssh -- "$robot_host" "python3 - '$remote/experiment.json'" <<'PY'
import json
from pathlib import Path
import sys

try:
    with Path(sys.argv[1]).open(encoding='utf-8') as stream:
        manifest = json.load(stream)
    if manifest['capture']['state'] != 'finished':
        raise ValueError('会话的采集收尾尚未确认')
except (OSError, ValueError, KeyError, TypeError) as error:
    raise SystemExit(f'无法下载会话：{error}')
PY
fi

mkdir -p -- "$destination"
exec rsync -az --checksum --itemize-changes --human-readable --info=progress2 -- \
    "$robot_host:$remote/" "$destination/"
