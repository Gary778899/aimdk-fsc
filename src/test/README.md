# 自定义测试工具

ROS 2 包名为 `aimdk_test`，使用 `ament_python`。可执行脚本和辅助模块位于
`aimdk_test/`，包配置位于当前目录。`head_test` 和 `arm_test` 默认只读；
原有的运动逻辑和显式 `--execute` 开关保持不变。

由用户同步工作区后，在机器人开发板的 Humble 容器中，从工作区根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to aimdk_test aimdk_functions
source install/local_setup.bash
ros2 run aimdk_test head_test --help
ros2 run aimdk_test arm_test --help
```

如果使用自定义构建、安装目录，请沿用对应的 colcon 参数，并 source 对应的
`local_setup.bash`。默认日志写入检测到的工作区 `log/`；工作区无法检测时，
使用 `--log-dir` 指定路径。

新增脚本放入 `aimdk_test/`，在 `setup.py` 的 `console_scripts` 中注册
`命令名 = aimdk_test.模块名:main`，并在 `package.xml` 声明新增 ROS 依赖。
本包内模块互相导入时使用 `aimdk_test` 前缀。

维护者邮箱和许可证目前为占位信息，正式发布前需替换为实际值。
