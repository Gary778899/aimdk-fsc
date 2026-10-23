# 自定义功能工具

ROS 2 包名为 `aimdk_functions`，使用 `ament_python`。脚本位于
`aimdk_functions/`，包配置位于当前目录。

由用户同步工作区后，在机器人开发板的 Humble 容器中，从工作区根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to aimdk_test aimdk_functions
source install/local_setup.bash
ros2 run aimdk_functions volume_control --help
ros2 run aimdk_functions volume_control get-volume
```

如果使用自定义构建、安装目录，请沿用对应的 colcon 参数，并 source 对应的
`local_setup.bash`。

新增脚本放入 `aimdk_functions/`，在 `setup.py` 的 `console_scripts` 中注册
`命令名 = aimdk_functions.模块名:main`，并在 `package.xml` 声明新增 ROS 依赖。

维护者邮箱和许可证目前为占位信息，正式发布前需替换为实际值。
