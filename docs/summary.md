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


建议把**一次示教尝试作为一个独立的会话目录**。实机上用一个入口管理这次录制，用一个命令把整个目录下载到本地。目录内部继续用 MCAP 保存原始 ROS 数据，用 JSONL 保存 XR 和控制事件，用一份 `session.json` 串起配置、文件和实验结果。

现有的 `data/`、`log/` 分开保存可以作为过渡。第一轮先给采集和遥操加同一个会话编号，验证下载后能准确找到同一次尝试的全部文件。目标形态是产物直接归入同一个会话目录，日常靠目录就能找到证据。

## 每次示教记录什么

| 内容 | 建议记录 |
| --- | --- |
| **实验条件与结果** | 任务、实验组、重复次数、初始姿态、工件和夹具摆放、工具与负载、补偿开关、任务阶段、成功／失败／中止和原因。插孔实验尤其要留孔轴、初始偏差和成功判据。 |
| **机器人本体与反馈** | 机型、URDF、实际 SDK／固件版本、运控模式及变化、带关节名称的实际位置、夹爪开度和故障。速度、电流、`effort`、腰部和 IMU 按任务选录。力估测实验保留原始字段及其已核实的物理含义。 |
| **遥操输入与动作** | PICO 位姿、Grip 和扳机；映射后的 TCP 目标、补偿前理想关节目标、实际发出的原生上肢指令和夹爪目标。HAL 指令与关节反馈各自保留，后续可以检查命令经过运控后发生了什么。 |
| **运行配置与版本** | 实际命令和参数、运行代码版本或摘要、生效的映射和采集配置、TCP 配置、补偿参数。用同一份运行配置可以重建实验条件。 |
| **时间与质量** | 各路源时间、接收时间、控制发送时间、输入与命令序号、开始和结束时刻。收尾记录缺流、丢弃和读回结果；离线配对时保留配对时间差和缺样标记。 |
| **图像与标定** | 按任务选录相机。启用时记录设备和视角、分辨率、帧率、内参；需要跨相机或机器人坐标系分析时，再关联对应的外参标定。 |
| **诊断日志** | 保留本次尝试的控制事件、故障、退出原因和必要的终端输出。colcon 构建日志等全局日志仍放在现有位置。 |

最值得补的三项是**原始 XR 输入、实验条件与结果标签、跨来源的时间关联**。有了这三项，后续才能回答操作者做了什么、机器人收到了什么，以及这次动作为什么被判为成功或失败。

## 会话目录怎么组织

```text
data/experiments/peg_in_hole/20261119_143025_rep01/
  session.json       # 会话编号、条件、版本、文件清单、结束状态与读回结果
  raw/               # MCAP 和 rosbag2 元数据
  xr.jsonl           # PICO 输入与输入事件
  control.jsonl      # IK、实际发送目标、反馈关联与故障
  notes.md           # 可选，现场异常和需要人工解释的观察
```

**每次尝试一份 `session.json` 就够。** 固定的实验方法、装置定义和成功判据写在一份可版本管理的实验方案里，会话清单引用其版本。`notes.md` 只在有现场补充时创建。重试生成新会话，成功和失败的尝试都能独立回查。目录里保存原始产物，轨迹和分析结果可以在本地生成并记录来源。

实机录制入口负责创建目录、分配会话编号、启动采集和关联本次遥操日志，结束时关闭并写好清单。**录制与运动授权各自显式**，实机运动继续沿用现有 `--execute`。下载入口按会话编号取整个已收尾的目录。首版可以复用现有采集器、遥操入口和 rsync，先用一次短示教验证目录、清单和读回，再把手动步骤收敛成两个命令。


建议保留现有工作区分层和 ROS 包，优先打通一轮可复现的双臂示教实验。 当前最影响科研推进的是三件事：采集、XR 和控制记录能否归到同一次实验，数据能否证明实际发出了什么动作，以及分析结果能否用同一批原始数据重建。先用一个短会话打通这条链路，再做包内重构和全量性能优化。

本次由三个子代理分别调查部署与复现、包边界、采集链路，我复核了关键源码、文档和已有会话摘要。审查以当前工作树为准，包含采集包已有的四个未提交文件。仓库未作修改。验证范围为本地静态审查及历史产物读取，容器构建、项目测试和当前版本的真机复验均待执行。

## 一、先解决影响实验结论的问题

1. 给每次实验一个共同编号和可复核的运行清单。 目前 MCAP 和日志分别下载，XR 日志已经指向对应的控制日志，轨迹导出也会保存源日志哈希，但还缺一个共同编号把这些产物与实验条件连起来。历史补偿实验中，同一个 pose1_rep1 标记下出现了多次启动和失败，两组补偿开关的目标姿态也不同，直接比较整段 RMSE 会混入姿态与运动过程的差异。建议复用现有 session.json 和日志元数据，为每次尝试记录实验编号、组别、起始参考、工具与负载、结束原因，以及 MCAP、XR、控制日志和分析结果的相对路径与校验值。代码来源也应一起记录：日常 rsync 排除 .git，采集器却在 CMake 配置时用 Git 识别版本，已有会话的 code_version_at_build 确实为 unknown。可以在同步时生成源码摘要，在构建时记录采集二进制对应的源码，在运行时记录实际加载的 Python 文件和配置，兼容现有 --symlink-install 流程。 历史实验分析 (docs/teleop_tracking_error_analysis.md:36) · 同步任务 (.vscode/tasks.json:117) · 构建版本实现 (overlay/src/data_collection/CMakeLists.txt:28) · 历史会话摘要 (data/data_20261112_205504/session.json:229)
2. 先定义示教数据里的动作，再做 ACT 导出。 当前 manipulation 预设采集 HAL 手臂指令、反馈和图像，自研遥操实际发布到 /mc/upper_body_command。控制日志的 command_published 已经保存了发送关节目标、夹爪开度、理想目标、补偿和时间信息，现有轨迹导出也从这里读取。首版可以直接以这份日志作为发送动作的来源，MCAP 提供原始观测，HAL 命令保留为独立观测量。用一个短会话验证动作与图像、关节和夹爪反馈的配对，输出实际配对时间差、缺样标记、任务阶段和成功／中止结果。等配对规则稳定后，再决定是否把原生上肢指令及实际 MC 模式一并纳入采集预设。 采集预设 (overlay/src/data_collection/config/collection_profiles.json:17) · 原生发布入口 (overlay/src/functions/aimdk_functions/upper_body_execution.py:133) · 发送事件 (overlay/src/functions/aimdk_functions/upper_body_teleop_execution.py:194) · 轨迹导出 (overlay/src/functions/aimdk_functions/upper_body_trajectory_core.py:95)
3. 让下载验收覆盖实际文件和任务所需的数据。 data_complete 表示正常收尾、每路有样本且采集队列零丢弃。默认的本地 inspect_collection 只读 session.json，因此只复制摘要也可能返回成功。建议先补本地文件存在性检查，明确报告摘要检查与实际消息读回的结果，再复用 Humble 的 --verify-mcap 做完整读回。研究层的检查放在离线导出，按实验预设报告各流覆盖时长、最大缺口和配对偏差。历史 full 会话在约 30 秒内丢了 9,466 条消息，占提交量的 16.41%，手臂反馈、指令和 IMU 都受影响。当前 full 已增加来源，应先用同一工况比较无图像、manipulation 和 full，结合已有的序列化、排队和写盘计时定位瓶颈。首批示教可先用满足任务需求的较小预设。 读回实现 (overlay/src/data_collection/scripts/inspect_collection.py:332) · 历史全量分析 (exports/collection_pair_20261112/report.md:7) · 在线耗时指标 (overlay/src/data_collection/README.md:212)

## 二、保留包边界，调整已经出现的耦合

| 部位 | 建议 |
|---|---|
| remote_control 与 aimdk_functions | 保持 XR 输入、Grip 和映射与 AimDK 解耦，机器人 IK 和执行留在 aimdk_functions。现场入口现在从两个命令模块导入 Recorder 和 replay_frames。把这两段共用逻辑各自移到所属包的普通模块，现有命令继续调用，复用 Grip 边沿、双侧联合提交、空解恢复的现有回归用例。 现场入口 (overlay/src/functions/aimdk_functions/upper_body_teleop_observe.py:11) · 共用帧处理 (overlay/src/functions/aimdk_functions/upper_body_teleop_replay.py:113) |
| data_collection 与腕部相机 | 采集入口和独立相机入口已经共用设备配置，但各自组装序列号、命名空间和驱动参数。让配置所属包提供一处参数构造，采集入口继续决定启用哪侧、哪类流，并保存生效配置。MCAP 保存原始 ROS 消息，控制日志保留映射、IK、调度和实际使用的反馈快照，用共同实验编号关联。 采集启动 (overlay/src/data_collection/launch/data_collection.launch.py:43) · 独立相机启动 (overlay/src/functions/launch/wrist_cameras.launch.py:13) |
| aimdk_test 与 locomotion_wrapper | aimdk_test 继续承载独立的头部实验和上肢只读诊断，两个入口的规模用现有包足够。locomotion_wrapper 当前是未部署的标准 ROS 状态适配，默认配置面向腿部观测。等全身策略或其他消费者明确需要标准话题后，再决定启用范围和验证条件。 实验入口 (overlay/src/test/setup.py:22) · 观测包范围 (overlay/src/locomotion_wrapper/README.md:3) |

## 三、优先处理的代码和流程细节

- 修正遥操分析的阶段归类。 局部 IK 无解后会冻结当前发布理想目标，目标缓存仍沿用上一个已接受输入序号。分析器主要按该输入的 enabled 和补偿进度划分阶段，所以这段冻结可能算进 tracking 或 ramp。现有日志已记录保持原因和发布暂停标志，建议分别统计 Grip 保持、空解保持、恢复和跟踪，将补偿渐入作为独立标记。这会直接改善静态误差、动态滞后和补偿对照的取样。 保持逻辑 (overlay/src/functions/aimdk_functions/upper_body_teleop_observe.py:241) · 分析分段 (overlay/src/functions/aimdk_functions/upper_body_teleop_analyze.py:102)
- 区分 MCAP 关闭与最终摘要落盘。 STOP 先设置 closed_，随后写最终 session.json。如果最后这次写入失败，重试可能因为 closed_ 已为真而提前返回，磁盘上仍是录制开始时的摘要。建议把摘要落盘作为单独的收尾状态，让重复 STOP 可以补写，并保留首次错误供排查。这是源码推导出的失败路径，待容器验证。 会话收尾实现 (overlay/src/data_collection/src/session_writer.cpp:481)
- 把当前状态收敛到总计划，把复现命令留在包说明。 轨迹格式、倍速、起点过渡和 50 Hz 选点已有实现及文档，总计划仍将其中几项列为待定。建议将状态更新为“已实现，当前版本待真机验收”，明确 upper_body_teleop_replay 用于离线重算 XR→IK，upper_body_trajectory run 用于已发送轨迹的预览与执行。部署继续沿用用户预览、同步、容器验证的流程。下次新容器验证时，将实际补装的采集依赖和宿主挂载版本纳入环境记录，减少复用旧容器带来的差异。 阶段计划 (docs/dev_plan.md:85) · 现有轨迹契约 (overlay/src/functions/README.md:182) · 容器复用逻辑 (docker/run.sh:5) · 采集依赖 (overlay/src/data_collection/README.md:48)

实施顺序：第一轮把共同实验编号、实际版本和下载读回接起来，完成一次含双臂和夹爪动作的短录制、离线导出、预览与真机回放，产出一份能追溯到原始记录的样例。第二轮用相同起点、理想目标和到达过程做补偿开关配对。现有轨迹回放重发已发送目标且关闭额外补偿，适合验证回放链路，补偿配对则应固定补偿前的理想目标。最后根据采集计时和任务所需模态决定全量吞吐优化，再推进正式示教集和 ACT 训练。 现有回放语义 (overlay/src/functions/README.md:193) · 补偿配对方案 (docs/teleop_tracking_error_experiments.md:135)