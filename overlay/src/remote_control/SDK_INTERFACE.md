# XR SDK接入接口

更新日期：2026-11-02。状态：代码及静态检查，待Orin部署验证。

`remote_control.sdk`直接通过ctypes调用只读C ABI，复制同一回调的完整JSON；
Python包`xrobotoolkit-sdk`仅用于定位随包`libPXREARobotSDK.so`，不调用其分字段getter。
也可用`--sdk-library`指定匹配架构的库。日志记录实际库路径及SHA256。

ABI依据：[PXREARobotSDK.h](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/vendor/include/PXREARobotSDK.h)。
固定结构为32字节设备标识和16352字节JSON字符数组；使用PXREAInit/PXREADeinit及服务／设备／JSON回调。
部署前将头文件和库版本固定并核对ABI。数据解析依据[上游回调实现](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/bindings/py_bindings.cpp)。

## 输入配置

复制`config/input.example.json`并填写协议约定，由`--input-config`指定。
以下为待现场核对的格式示例：

```json
{
  "quaternion_order": "xyzw",
  "position_scale_m": 1.0,
  "reference_space_id": "recording-session-1",
  "validity_field": null,
  "valid_values": [],
  "reference_space_field": null
}
```

- `quaternion_order`：原始四元数顺序，xyzw或wxyz；位置缩放由`position_scale_m`转换为m。
- `validity_field`／`valid_values`：单侧Controller对象的跟踪状态字段及有效值，字段缺失或配置null时保留未知。
- `reference_space_field`：Tracking.value中的参考空间标识字段；配置null时使用显式reference_space_id。
  无上游重定位标识时，应用重定位后结束录制，以新reference_space_id重新录制。

## 命令与日志

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
