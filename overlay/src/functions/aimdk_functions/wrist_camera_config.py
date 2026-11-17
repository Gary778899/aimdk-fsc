"""读取左右腕部 D405 共用的设备绑定与图像流配置。"""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def load_camera_config():
    """返回已安装的腕部相机配置字典，包含命名空间、左右序列号和驱动参数。"""
    config_path = (
        Path(get_package_share_directory('aimdk_functions'))
        / 'config' / 'wrist_cameras.json'
    )
    return json.loads(config_path.read_text(encoding='utf-8'))
