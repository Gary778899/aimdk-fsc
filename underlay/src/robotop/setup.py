from setuptools import find_packages, setup

package_name = "robotop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agi",
    maintainer_email="agi@local",
    description="Robot debug TUI (status / logs / comm / faults).",
    license="TODO",
    entry_points={
        "console_scripts": [
            "robotop = robotop.cli:app",
            "orin_monitor_node = robotop.orin_monitor_node:_cli_entry",
        ],
    },
)
