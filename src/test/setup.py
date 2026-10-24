"""Install the custom robot test tools as a ROS 2 Python package."""

from setuptools import find_packages, setup


package_name = 'aimdk_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gary',
    maintainer_email='gary@example.com',
    description='Custom head and arm diagnostic and acceptance tools for AimDK.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'head_test = aimdk_test.head_test:main',
            'arm_test = aimdk_test.arm_test:main',
            'dual_arm_control = aimdk_test.dual_arm_control:main',
            'shoulder_direction_test = aimdk_test.shoulder_direction_test:main',
            'gripper_close = aimdk_test.gripper_close:main',
        ],
    },
)
