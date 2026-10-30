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
            'dual_arm_control = aimdk_functions.dual_arm_control:main',
            'arm_compensation_probe = aimdk_functions.arm_compensation_probe:main',
            'upper_body_kinematics_check = aimdk_functions.upper_body_kinematics_check:main',
            'upper_body_move = aimdk_functions.upper_body_move:main',
            'upper_body_line = aimdk_functions.upper_body_line:main',
        ],
    },
)
