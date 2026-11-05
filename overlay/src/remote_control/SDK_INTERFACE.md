# XR SDK接入接口

更新日期：2026-11-02。状态：代码及静态检查，待Orin部署验证。

`remote_control.sdk`直接通过ctypes调用只读C ABI，复制同一回调的完整JSON；
Python包`xrobotoolkit-sdk`仅用于定位随包`libPXREARobotSDK.so`，不调用其分字段getter。
也可用`--sdk-library`指定匹配架构的库。日志记录实际库路径及SHA256。

ABI依据：[PXREARobotSDK.h](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/vendor/include/PXREARobotSDK.h)。
固定结构为32字节设备标识和16352字节JSON字符数组；使用PXREAInit/PXREADeinit及服务／设备／JSON回调。
部署前将头文件和库版本固定并核对ABI。数据解析依据[上游回调实现](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/bindings/py_bindings.cpp)。

## 输入配置

当前只读接入使用[config/input.json](config/input.json)，固定2026-11-02录制日志中的配置：
`quaternion_order=xyzw`、`position_scale_m=1.0`、`reference_space_id=pico-session-001`，
`validity_field`和`reference_space_field`为null，`valid_values`为空数组。
依据为`log/pico_input_001.jsonl`及同日两次`xr_watch`日志的metadata；实际轴向、距离与旋转方向仍待现场核对。

`xr_watch`默认读取同步后的源码配置，路径见[常驻输入测试](README.md#常驻输入测试)。
`setup.py`同时将配置安装到`share/remote_control/config/`。
其他输入约定可复制`config/input.example.json`填写，并用`--input-config`指定；`xr_record`始终显式指定该参数。

- `quaternion_order`：原始四元数顺序，xyzw或wxyz；位置缩放由`position_scale_m`转换为m。
- `validity_field`／`valid_values`：单侧Controller对象的跟踪状态字段及有效值，字段缺失或配置null时保留未知。
- `reference_space_field`：Tracking.value中的参考空间标识字段；配置null时使用显式reference_space_id。
  无上游重定位标识时，应用重定位后结束录制，以新reference_space_id重新录制。

## 命令与日志

2026-11-03的`log/xr_watch_20261103T124408_094273Z_f22b0cb9.jsonl`中，1764条Tracking回调
均包含`Head`及`Controller.left/right`，共享该回调的`timeStampNs`和本机接收时刻。
`Head.pose`为逗号分隔的3位置及4四元数，与手柄共用`position_scale_m`和`quaternion_order`配置。
`XRFrame.head`保存解析位姿，`head_status`原样保存`Head.status`；本次值为3，其有效性含义未核实，不用作门槛。
未开启Head时两字段为null；历史归一化帧缺少两字段时读回为null，维持`remote_control.xr.v1`可选字段扩展。
同一回调表示软件快照关联，头部与手柄的物理坐标一致性及朝向映射仍由现场验证。

常驻等待、实时状态显示及自动命名日志使用[xr_watch](README.md#常驻输入测试)。
下面的`xr_record`用于显式设备和输出路径的录制。

```bash
ros2 run remote_control xr_record --device-id 实际设备标识 --input-config /实际路径/input.json --output /home/agi/aimdk-fsc/log/xr_session.jsonl --duration 60
ros2 run remote_control xr_stats /home/agi/aimdk-fsc/log/xr_session.jsonl
```

每条原始回调记录callback_seq、接收单调时间、类型、原始status、设备标识、Base64字节副本及队列积压。
每个完整输入帧记录独立seq及关联callback_seq。队列按FIFO消费，其他设备只记录原始数据。
正常终止后排空原始回调；SDK终止失败时保留回调引用并记录失败。
源时间精度按安装版本核对；统计只使用本机接收间隔，不推算跨设备延迟。

JSONL schema为`remote_control.xr.v1`，事件包括metadata、sdk_event、xr_frame、session_invalidated、error、exit。
`recording.read_frames(path)`按文件顺序读回XRFrame，遇到异常、失联或源时间回退时报错停止。
`recording.summarize(path)`保留异常计数并输出P50/P95/P99/最大接收间隔、实际输入帧率和积压。
`recording_complete`核对有效帧、正常退出、异常及序号／时间完整性；统计命令退出0表示读回成功。

后续验证：检查ABI、动态库加载、设备标识、同帧快照、原始帧读回、断连记录及退出队列排空。
