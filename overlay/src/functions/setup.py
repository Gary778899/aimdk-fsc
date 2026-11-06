"""Install the custom robot utilities as a ROS 2 Python package."""

from setuptools import find_packages, setup


package_name = 'aimdk_functions'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy>=1.21'],
    package_data={
        'aimdk_functions.upper_body_kinematics': ['assets/*'],
        'aimdk_functions.upper_body_kinematics.vendor': ['assets/*.json'],
    },
    zip_safe=False,
    maintainer='gary',
    maintainer_email='gary@example.com',
    description='Custom robot utility commands for AimDK.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'volume_control = aimdk_functions.volume_control:main',
            'upper_body_control = aimdk_functions.upper_body_control:main',
            'upper_body_home = aimdk_functions.upper_body_home:main',
            'upper_body_trajectory = aimdk_functions.upper_body_trajectory:main',
            'arm_compensation_probe = aimdk_functions.arm_compensation_probe:main',
            'upper_body_kinematics_check = aimdk_functions.upper_body_kinematics_check:main',
            'upper_body_acceptance = aimdk_functions.upper_body_acceptance:main',
            'upper_body_teleop_replay = aimdk_functions.upper_body_teleop_replay:main',
            'upper_body_teleop_observe = aimdk_functions.upper_body_teleop_observe:main',
            'upper_body_teleop_stats = aimdk_functions.upper_body_teleop_stats:main',
            'upper_body_teleop_analyze = aimdk_functions.upper_body_teleop_analyze:main',
        ],
    },
)
