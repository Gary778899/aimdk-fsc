# Read-only state observer

本包保留在本地 `overlay/src/locomotion_wrapper/`，本轮不参与机器人同步或构建，
`scripts/build_overlay.sh` 不接受本包。分层部署流程见[根目录说明](../../../README.md)。
以下启动及测试命令仅供未来明确部署本包时参考，不表示本轮已构建或验收。

Built against this SDK's `aimdk_msgs`. This package contains subscriptions and
standard sensor state publishers only. It does not command joints or call motion
services.

```bash
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run locomotion_wrapper hardware_wrapper_node
ros2 run locomotion_wrapper joint_watch
```

Inputs default to `/aima/hal/joint/{head,arm,waist,leg}/state` and
`/aima/hal/imu/chest/state`. Outputs are `/humanoid/joint_states` and
`/humanoid/imu`. Input parameters are `{head,arm,waist,leg}_joint_state_topic`
and `imu_input_topic`; output parameters are `joint_state_topic` and `imu_topic`.
The chest IMU is retained from the old observer. It is not assumed equivalent to
the torso IMU used by the policy: explicitly select the physically correct input.
Do not remap an output onto its input.

For this WBC migration only the leg group is required by default
(`require_leg=true`; `require_head`, `require_arm`, `require_waist=false`). The
leg expected set defaults to the twelve names explicitly used by the old WBC
policy, not a guessed joint count. See `config/wbc_observer.yaml`. Other robot
models can override `leg_expected_names`.

Optional means an absent group may be omitted; an optional group that arrives is
still validated, and stale cached optional data suppresses the aggregate rather
than silently changing its contents. To enable complete head/arm/waist checks,
provide verified `{head,arm,waist}_expected_names` lists and set the matching
`require_*` flag. A required group with an empty expected list **never produces
valid output** and logs a configuration reason. Unknown optional groups may be
included without a completeness claim; the supplied WBC profile directs their
subscriptions to unused topics so that the default aggregate is legs only.

```bash
ros2 launch locomotion_wrapper observer.launch.py
# Or select a custom verified robot profile:
ros2 launch locomotion_wrapper observer.launch.py params_file:=/path/to/robot.yaml
```

`max_age_sec=0.1` bounds both source timestamp age on the ROS clock and receipt
age on the monotonic clock. `future_tolerance_sec=0.005` permits small future
clock offsets; `max_group_skew_sec=0.02` bounds inter-group source stamp spread.
These defaults are validation starting points, not measured robot guarantees.
ROS clock and vendor `header.stamp` must share a time base (including rosbag
playback with `use_sim_time`). No conversion from `meas_stamp` is guessed.

The aggregate uses the **oldest original `header.stamp`** among its groups,
never the timer's current time. This is a multi-source aggregate, not a hardware
synchronized sample: individual per-group timestamps and frames cannot all fit
in standard `JointState`; record the vendor topics if those fields are needed.
Its `frame_id` is left empty because joint names describe multiple body frames.
IMU output preserves the full original message including stamp and frame.
The 2 ms timer validates latest caches; unchanged caches are not republished.
Every new joint-group receipt can produce an aggregate while the other groups
remain within the configured age/skew bounds.

Missing/stale/invalid input suppresses the corresponding output and logs a
throttled reason. Validation rejects zero/invalid/future timestamps, group and
joint errors, duplicate/empty names, empty groups, non-finite measurements,
unavailable IMU components/covariances, and non-unit quaternion (squared norm
tolerance 0.01). IMU acceleration and angular velocity are required along with
orientation. There is no fallback, extrapolation, or normalization of bad data.
`joint_watch` uses the same default joint output topic and warns on receipt
timeout (`timeout_sec=1.0`).

Core tests instantiate message structs directly; no ROS node, DDS discovery,
robot connection or network permission is required:

```bash
colcon test --packages-select locomotion_wrapper
colcon test-result --verbose
```
