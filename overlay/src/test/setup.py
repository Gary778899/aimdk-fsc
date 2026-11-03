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
    description='Head experiments and read-only upper-body command diagnostics for AimDK.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'head_test = aimdk_test.head_test:main',
            'upper_body_command_watch = aimdk_test.upper_body_command_watch:main',
        ],
    },
)
