# Read-only data collection migration

当前构建、依赖、解释器与验收流程见 [Humble 操作指南](../../docs/humble_baseline.md)。以下命令在机器人开发板容器的工作区根目录执行，需先按指南完成干净构建。

Records `/humanoid/joint_states` (`sensor_msgs/msg/JointState`) and `/humanoid/imu`
(`sensor_msgs/msg/Imu`) to MCAP. The combined launch starts the read-only
`locomotion_wrapper` observer via its installed `observer.launch.py` profile;
`params_file:=...` overrides that observer profile. No actuator commands or motion service calls exist.

## Build dependencies

ROS 2 Humble (Ubuntu 22.04 container): rclcpp, sensor_msgs, rosbag2_cpp, rosbag2_storage, mcap_vendor,
rosbag2_storage_mcap. Native dependencies: pkg-config, libzmq3-dev, cppzmq-dev.
The CMake configuration needs real `libzmq.pc` and `zmq.hpp` development files;
a runtime-only `libzmq.so.5` is insufficient. No network download or dependency
installation occurs during the build. When authorized, install native development
packages with the system package manager, then build this package with colcon.

## Run

```bash
source /opt/ros/humble/setup.bash
source .venv-humble/bin/activate
source install-humble/local_setup.bash
ros2 launch data_collection_edge_agent robot_data_collection.launch.py bag_root_dir:=./rosbags
# Or launch only the recorder:
ros2 launch data_collection_edge_agent data_collection_edge_agent.launch.py
python tools/edge_agent_control.py START
python tools/edge_agent_control.py STOP
python tools/mcap_to_robot_csv.py rosbags/data_log_... --output robot_data.csv
```

`zmq_bind_address` defaults to `127.0.0.1`, port `5555`; both are configurable
launch arguments, as are the input topics. Controller `--endpoint` and
`--timeout-ms` configure the client. No command argument gives an interactive
recording session that attempts STOP on Ctrl-C / EOF. Requires `pyzmq`.

The wire protocol remains REQ/REP plain strings: START/STOP → ACK. Unknown commands,
file creation/closing failures, and prior write failures return `ERROR <reason>`.
Repeated START/STOP are idempotent. ACK for START confirms writer initialization,
not receipt of any sensor samples. A later write failure disables recording;
STOP reports that failure, and a successful new START clears it. No authentication
is added to the existing protocol; any remote listening address is opt-in.

The node waits for bind readiness before declaring successful construction,
joins its control thread, and closes the writer on destruction. Receive polling
and send timeout prevent idle/client-disconnect indefinite waits. File I/O itself
can still stall with a broken filesystem. Bag names include nanoseconds, PID,
and a process-local counter. Source message headers remain intact; rosbag storage
timestamps record node receipt time. Abrupt power loss is outside clean-close
behavior.

## CSV semantics and validation

The exporter has no pandas dependency. By default it uses the existing ROS 2
`rosbag2_py` runtime; optional `--reader rosbags` uses rosbags and embedded MCAP
schemas without a hardcoded Humble typestore. That optional backend requires a
separate `rosbags` install and was not tested where unavailable.

Output retains joint names and position/velocity/effort arrays as JSON, source
stamps and bag receipt stamps as exact integer nanoseconds, IMU orientation,
acceleration and angular velocity, and frame IDs. Empty optional JointState arrays
remain `[]`; malformed lengths, duplicate/empty names and nonfinite sensor values
fail explicitly. Zero source stamps remain zero; they are never silently replaced
with receipt times and should not be interpreted as synchronized hardware time.

The stable chronological one-to-one tolerance join consumes every sample once.
It deliberately does not promise the globally closest assignment: if the oldest
unconsumed pair falls within tolerance, it pairs them. It preserves duplicate
stamps and unpaired samples, with blank counterpart fields. `alignment_error_ns`
is joint source time minus IMU source time, present only on pairs. This is an
offline alignment aid, not hardware synchronization. Missing requested topics
are allowed if the other exists; valid empty streams produce a header-only CSV.
No implicit legacy topic substitutions occur; use `--joint-topic` / `--imu-topic`.
The current exporter holds selected samples in memory; split very large bags.

```bash
source /opt/ros/humble/setup.bash
.venv-humble/bin/python -m unittest discover -s tools/tests -v
```

Tests cover malformed joint data, one-sided/empty streams, duplicate stamps,
exact timestamp CSV encoding, actual MCAP serialization/readback/export without
DDS, protocol response handling and timeouts. An optional actual ZMQ IPC test is
explicitly skipped if sandbox restrictions deny sockets.
