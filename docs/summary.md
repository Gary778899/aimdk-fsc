截至 **2026-11-22**，主线已经拿到两条真机基线：**无相机、无夹爪的全身数据采集已通过 MCAP 读回，PICO 原始输入已通过独立 JSONL 录制。** 下一阶段是把这两条链路合到同一次示教中，验证 `xr_raw` 入包、命令与反馈配对、任务标注和整次会话下载。

## 已完成的开发与测试

| 范围 | 已实现 | 已验证 |
| --- | --- | --- |
| **部署与工作流** | 厂商 SDK、自研 overlay 和 RealSense 分层。已有预览同步、容器构建和数据下载入口。 | 11 月 20 日的 Humble 构建日志确认 `aimdk_test`、`remote_control`、`aimdk_functions`、`data_collection` 四个包均构建成功。 [构建日志](/home/gary/projects/aimdk-fsc/log/build_2026-11-20_14-07-58/events.log:912) |
| **全身采集** | 按预设录制关节反馈、HAL 指令、胸部和骨盆 IMU、头部和双腕图像。支持 START／STATUS／STOP、压缩 MCAP、写盘统计和读回校验。 | `full-no-cam` 挥手会话约 **51.8 秒、233,206 条消息、采集队列零丢弃**，MCAP 读回通过。唯一零样本话题是夹爪指令，因此完整性检查返回 `1`。夹爪反馈有 **10,333 条**，全部上报 `NONE` 和空数组。 [会话摘要](/home/gary/projects/aimdk-fsc/data/wave_baseline/20261120_142431/session.json:180) · [读回报告](/home/gary/projects/aimdk-fsc/data/wave_baseline/20261120_142431/readback.json:259) |
| **PICO 原始输入** | 已实现 SDK 服务接入、原始回调与完整双手帧录制、接收时序统计，以及可选的 `xr_raw` ROS 发布。 | 约 60 秒的独立录制得到 **5,352 帧、约 90 Hz**，正常收尾。三份录制共 **6,607 帧**，左右 `trigger` 和 `grip` 的原始值与解析值逐帧一致。两侧 `trigger` 和左 `grip` 均记录到 `1.0`。右 `grip` 满按尚未覆盖，部分日志存在双手位置同时归零的片段。 [60 秒基线](/home/gary/projects/aimdk-fsc/log/xr_baseline_20261120_143728.jsonl:1) · [Grip 复验](/home/gary/projects/aimdk-fsc/log/xr_grip_check_20261120_144210.jsonl:1) |
| **上肢控制与遥操** | 已实现关节、TCP、夹爪控制和 Home 回位，以及 PICO 双臂 IK、连续遥操、Grip 边沿处理、局部 IK 无解保持与恢复、位置偏置补偿、已发送轨迹导出和回放。 | 既有版本有上肢控制、现场遥操和跟随误差日志。已完成名义 TCP 误差分解，以及静态保持和补偿开关的探索性分析。当前版本仍需完整复验。 [功能入口](/home/gary/projects/aimdk-fsc/overlay/src/functions/README.md:67) · [误差实验记录](/home/gary/projects/aimdk-fsc/docs/teleop_tracking_error_analysis.md:3) |
| **单次示教会话** | 已实现同目录保存 MCAP、XR 与控制日志、实验清单、结果标签，支持 `xr_raw` 对账、时间配对导出和按会话下载。 | 已完成本地静态核对，目前下载的产物覆盖独立 MCAP 和独立 XR 两条链路。整次示教的真机产物待取得。 [会话操作与验收](/home/gary/projects/aimdk-fsc/overlay/src/data_collection/README.md:190) |
| **相机** | 双腕 D405 独立启动、单帧保存，以及头部和双腕图像的连续 MCAP 采集已接入。 | 双腕彩色单帧已在真机保存并下载；历史 `full` 会话能读回多模态数据，但队列丢弃 **9,466 条（16.41%）**，包括图像和关节等流。 [相机验证](/home/gary/projects/aimdk-fsc/overlay/src/functions/README.md:36) · [历史全量报告](/home/gary/projects/aimdk-fsc/exports/collection_pair_20261112/report.md:5) |

## 后续开发和测试顺序

1. **现在可做：补齐右 Grip 和 PICO 跟踪观察。** 继续用 `xr_record`，分别满按、保持、松开左右 Grip，确认右侧能跨过 `0.9` 并回落。另录一段手柄持续可见的位姿数据，记录双手是否仍会同时归零。挥手采集基线已有证据，无需为了退出码重录。下一次同工况采集可排除 `hand_commands`，保留 `hands` 记录无夹爪状态。 [映射阈值](/home/gary/projects/aimdk-fsc/overlay/src/remote_control/config/mapping.json:9) · [采集完整性规则](/home/gary/projects/aimdk-fsc/overlay/src/data_collection/README.md:170)
2. **夹爪装好后优先：验证 `xr_raw` 入 MCAP。** 用 `record_trial.py` 做一次正常只读录制和一次有有效 Tracking 后的只读断连录制。核对目标头显、完整双手帧、原始 SDK 回调与 JSONL 对账、断连事件和退出状态。现有 `upper_body_teleop_observe` 的只读模式也要求两侧有效 `CLAW` 反馈。若要在无夹爪时提前做这项验收，需要给独立 XR 录制入口补一个原始话题发布选项。 [夹爪反馈要求](/home/gary/projects/aimdk-fsc/overlay/src/functions/aimdk_functions/dual_arm_control_core.py:136) · [只读录制与读回](/home/gary/projects/aimdk-fsc/overlay/src/data_collection/README.md:190)
3. **跑通一次无图像短示教与轨迹回放。** 录一段含双臂和夹爪动作的短示教，核对实际发送目标能追溯到 XR 输入和反馈，完成标注、时间配对和按会话下载。再用 `upper_body_trajectory` 导出、预览并在机器人回放，记录起点衔接、夹爪动作和实际跟踪误差。 [示教验收](/home/gary/projects/aimdk-fsc/overlay/src/data_collection/README.md:253) · [轨迹回放验收](/home/gary/projects/aimdk-fsc/overlay/src/functions/README.md:182)
4. **补齐相机负载和跟随质量实验。** 相机先验收单流，再验收实际任务组合，最后复测 `full`，结合逐流缺口、图像与内参配对、队列丢弃及在线耗时确定采集配置。跟随质量按相同起点、理想目标和到达过程做补偿开关配对，补齐动态滞后分析。实体 TCP 精度待独立测量。 [采集耗时诊断](/home/gary/projects/aimdk-fsc/overlay/src/data_collection/README.md:286) · [跟随误差实验](/home/gary/projects/aimdk-fsc/docs/teleop_tracking_error_experiments.md)
5. **后续研发：示教数据集、ACT 和力估测。** 固定插孔任务的工况、成功判据、观测和动作表示，建立可追溯的数据集与训练验证划分，再完成 ACT 训练和实机执行。三维力估测并行核实 `effort` 的物理含义、坐标与参考测量方案。递奖状、抓瓶和插孔的任务级验证随后展开。 [后续研究路线](/home/gary/projects/aimdk-fsc/docs/dev_plan.md:59)

还有一项影响复现的记录需要补齐：11 月 20 日会话里的代码和固件版本均为 `unknown`，记录的 SDK 版本字符串也与本地 SDK 副本不同。下轮录制时核对机器人实际安装前缀和对应版本文件，把结果与会话一起保存。 [实机会话版本字段](/home/gary/projects/aimdk-fsc/data/wave_baseline/20261120_142431/session.json:152) · [本地 SDK 版本文件](/home/gary/projects/aimdk-fsc/underlay/version:1)

本次核对了本地工作树、下载的构建日志、采集读回报告和 XR 原始日志。未修改文件，未执行新的实机测试。