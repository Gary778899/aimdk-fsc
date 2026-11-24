#!/usr/bin/env python3
"""检查采集会话摘要，并可在 ROS 环境读回 MCAP。"""

from pathlib import Path
import sys


if __name__ == "__main__":
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / "data_collection" / "readback.py").is_file():
        sys.path.insert(0, str(source_root))
    from data_collection.readback import main

    sys.exit(main())
