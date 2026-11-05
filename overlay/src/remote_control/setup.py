"""安装与机器人SDK解耦的XR输入、映射及会话工具。"""

from setuptools import find_packages, setup


setup(
    name='remote_control',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/remote_control']),
        ('share/remote_control', ['package.xml']),
        ('share/remote_control/config', [
            'config/mapping.example.json', 'config/mapping.json',
            'config/input.example.json', 'config/input.json']),
    ],
    install_requires=['setuptools', 'numpy>=1.21'],
    zip_safe=False,
    maintainer='gary',
    maintainer_email='gary@example.com',
    description='XR input frames, relative pose mapping and teleoperation sessions.',
    license='TODO: License declaration',
    entry_points={'console_scripts': [
        'xr_record = remote_control.cli:record_main',
        'xr_watch = remote_control.watch:watch_main',
        'xr_stats = remote_control.cli:stats_main',
    ]},
)
