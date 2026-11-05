# 遥操跟随误差定位实验

更新日期：2026-11-05。适用X2 Ultra、SDK 1.1、ROS 2 Humble、URDF v1.3.0、PICO 4 Ultra。
状态：实验尚未执行。已在`d4976f1`基础上补充反馈关联、发布调用时间、独立身体记录与离线分析入口，待机器人容器验证。
用户确认当前没有外部位姿测量设备，首轮使用现有日志与静态保持实验；先完成[重构后回归](dev_teleop_part2.md#后续验证顺序)。
命令及异常处理沿用[功能包说明](../overlay/src/functions/README.md#遥操持续执行)，部署和日志下载沿用根目录流程。

## 1. 目标与当前证据

按“求解 → 发送 → 关节反馈 → 模型TCP → 实体TCP → 世界坐标”逐层定位。
首轮回答三个问题：模型内IK是否产生偏差、关节误差是否主要来自滞后、补偿是否改善理想目标的跟随。
实体运动学与世界坐标系误差暂保留为待测项。

已核对的[最近完整执行日志](../log/teleop_no_comp_20261103_205907/upper_body_1793711187927434632_50724dcb.jsonl)
实际开启补偿，1396条发布、560条IK接受记录，正常结束。
关节反馈相对补偿后发送目标的最大绝对误差为9.78°；右臂IK模型内位置残差最大约2.60×10⁻⁸m。
[已有TCP分析](../exports/teleop_tcp_tracking_20261103_2119/report.md)中，反馈FK相对理想目标的全程位置RMSE为14.67mm，
接管5秒后RMSE为10.71mm、最大18.66mm；相对发送目标的全程RMSE为43.29mm。
该分析含静态偏差、补偿渐入及动态滞后，没有分离各项成因。

## 2. 统一比较量

`B`表示`torso_link`，`W`表示实验中的固定世界坐标系；`F(q)`为同一名义模型与`gripper`工具配置的FK。

| 量 | 来源／定义 | 用途 |
| --- | --- | --- |
| `T_des` | 已接受输入对应的`tcp_target`，在B系表达 | 原始任务目标 |
| `q_ik` | `ideal_arm_rad` | 补偿前理想关节目标 |
| `q_send` | `arm_rad` | 实际发布关节目标 |
| `q_fb` | `actual_arm_rad`及原始arm反馈 | 实际关节反馈 |
| `b` | `q_send-q_ik`，与补偿日志逐项核对 | 实际引入的偏置 |
| `e_send` | `q_fb-q_send` | 对发送目标的关节误差 |
| `e_ideal` | `q_fb-q_ik=e_send+b` | 对理想关节目标的误差 |
| `e_ik` | `F(q_ik)`与`T_des`的差 | IK模型内残差 |
| `e_tcp` | `F(q_fb)`与`T_des`的差 | B系名义TCP任务误差 |

位置报告有符号XYZ误差及三维模长；姿态报告旋转矩阵间的最短旋转角，不能直接相减欧拉角。
关节报告每轴有符号均值、RMSE、P95绝对值和最大绝对值，左右分别统计。
位置与姿态使用同一个工具变换；历史单次`upper_body_control tcp`默认腕原点，不能直接与遥操`gripper`结果混用。

对同一时刻的B系位置，可作如下精确分解：

```text
F(q_fb).p - p_des
  = [F(q_ik).p - p_des]                  IK残差
  + [F(q_send).p - F(q_ik).p]             补偿引起的命令几何变化
  + [F(q_fb).p - F(q_send).p]             反馈相对发送目标的几何变化
```

三项相加复原名义TCP任务误差。三项的模长不能相加解释为误差贡献比例，补偿与跟踪项可能方向相反。
开启补偿后，即使`e_send`较大，只要`e_send+b`和`e_tcp`减小，补偿仍可能有益。
此分解是几何记账，动态片段的时序与因果关系另按实验B分析。

## 3. 数据与时间对齐

1. 保存原始XR、`mapping_candidate`、`tcp_target`、`ik_accepted`、`command_published`、arm／HAL命令反馈、模式及退出记录。
   开启补偿时同时保存IMU、腰反馈、补偿配置、`blend`、重力估计、原始／限幅偏置和饱和标记。
2. 每条发布用`input_seq`关联已接受目标。首次零增量发送的序号为空，单列启动段。
   以日志metadata确定补偿开关和代码摘要，目录名与文字负载说明仅作备注。
3. 原始反馈保留源时间、单调接收时间和serial；同一反馈被多次发布快照复用时单列，避免把重复反馈当成独立采样。
4. 新`command_published`直接保存arm反馈serial、源时间和单调接收时间，与所用关节向量来自同一快照。
   历史日志缺少关联时只能初步比较，存在多解的记录标记配对不确定；单凭相邻行不能确定采用的反馈。
5. 当前发布前取得的`q_fb`早于本次新命令的执行，`tracking_error_rad`是同时刻新目标差值。
   同时分析对上一发送目标的误差，并结合完整命令历史估计延迟；上一帧也不能自动等同于生效命令。
6. 首轮用本机发送／接收时间估计整条命令至反馈的表观滞后；传感器源时间与ROS命令时间的时钟关系核实后才做源时间对齐。
   记录软件传输、采样与运动响应混合的局限。禁止将头显源时钟直接与本机单调时间相减。

`publish_begin_monotonic_s`和`publish_end_monotonic_s`夹住发布调用；`command_stamp_s`为消息构造时ROS时间，
JSONL的`monotonic_s`晚于publish，三者不能互换，也不代表机器人内部执行时刻。
`input_age_s`对应主线程已选帧，发送目标的输入年龄须从该目标的`input_seq`重建。
关闭补偿时添加`--record-body`记录身体原始消息；该模式的IMU／腰不参与运动校验。
开启补偿时IMU／腰仍使用既有必需反馈检查，原始数据缺失与异常处理按功能包契约执行。

## 4. 实验A：现有日志的IK与误差分解

**操作**

1. 从最近完整会话提取每侧已接受目标、理想关节、发送关节和反馈；按日志模型摘要及工具变换重算FK。
2. 校验输入关联与关节槽位，并比较重算IK残差与`ik_accepted`记录。
3. 分别画`e_ik`、`b`、`e_send`、`e_ideal`和三项TCP位置分解；标出首次Grip、重按、补偿渐入和目标突变。
4. 先按接管起始段、`blend<1`、`blend=1`分组，再在每组中区分目标保持和运动。
   静态段须由`q_ik`实际恒定确认；不能仅以“手柄看起来没动”判定。

**判别**

- IK残差在问题片段内始终远小于任务误差，说明该片段的同模型数值求解不能解释厘米级偏差。
- 若模型内残差本身增大且与误差峰值同步，定位相应输入、SEW、分支和姿态，再单独复现求解。
- 同模型求解与FK一致仅评价算法内部一致性；实体模型准确性留给实验E。

**输出：** 逐发布误差表、各分段统计、时间曲线与首个异常片段，保留输入序号及命令序号。

## 5. 实验B：时序与动态滞后

先离线分析实验A的数据，再用下述现场动作补足覆盖。

**操作**

1. 关闭补偿，右臂单侧接管，执行“保持 → 沿一个方向缓慢移动 → 松开Grip保持 → 反向移动 → 保持”。
   松开Grip后PICO继续发送，机器人继续发布保持目标，不以关闭Send代替静态保持。
2. XYZ平移分别记录；第一轮保持姿态。慢速／较快两组使用相近空间路径，记录实际目标速度和幅度。
   左臂复测相同结构，双臂留在单侧结论后复测。
3. 对运动片段计算未经移位的`e_send`，再比较反馈与此前发送命令历史。
   在非周期、含正反向运动的片段中拟合一个非负表观延迟，同时拟合常值偏差；搜索上界由停走响应决定，不能任意扩大以压低残差。
4. 用另一段动作验证拟合延迟，报告对齐前后残差及接收年龄；无明显运动的关节不参与延迟拟合。
   对命令历史用零阶保持重建，在实际反馈接收时刻直接评价，不插值反馈，也不跨越命令缺口。

**判别**

- 移位后动态残差明显下降，较快动作误差增大，而终点保持误差较小：支持滞后是主要动态因素。
- 停止运动后误差仍持续：转入静态偏差和补偿分析。
- 原始接收年龄、选帧年龄或发布间隔同时增加：进一步区分调度与机器人响应，不能将全部拟合延迟归于机械控制。

**输出：** 表观延迟、分轴偏差、运动／保持误差、输入至目标延迟及发布间隔分布。

## 6. 实验C：无补偿静态偏差

**固定条件：** 夹爪开度固定，不持物；记录实际安装工具，维持原厂站立与原生上肢控制。
保持躯干没有主动运动，IMU与腰变化仍需观察记录。

**操作**

1. 右臂先选三个已检查姿态：自然屈肘、向前伸出、侧向展开。保存实际`q_ik`与名义TCP，作为姿态编号。
2. 每次到达姿态后松开Grip，继续发送XR并保持10秒；最后5秒作为预定分析窗口，同时检查反馈是否仍有趋势。
   有趋势时报告该段未达静态，延长观察或作为过渡段单列，不用末帧代表静态。
3. 每个姿态重复三次，从正反方向各至少一次到达，以观察方向相关偏差；先右臂，再左臂。
4. 统计`q_fb-q_ik`、名义TCP任务误差及HAL命令与发送命令的差异，记录保持中的身体姿态变化。

**判别**

- IK残差小，保持后关节偏差稳定：支持关节执行／保持环节存在静态偏差。
- 偏差随手臂伸展、重力力臂明显变化：支持重力相关假设，仍需补偿配对检验。
- 从不同方向到达同姿态时残差不同：提示摩擦、回差或控制历史影响，不能仅拟合统一刚度解释。
- HAL命令与原生发送目标不一致是运控内部处理的调查线索；其物理语义与延迟仍需单独核实。

## 7. 实验D：补偿是否有效、为何不足

当前遥操使用`b=blend×clip(τg/40, ±12°)`，每侧首次Grip后默认5秒渐入，重按不重置。
估计重力来自骨盆IMU及腰反馈；遥操入口附加载荷固定为0，模型仅计算七个臂连杆，安装夹爪／工具的质量不自动计入。
实现见[PositionCompensation](../overlay/src/functions/aimdk_functions/upper_body_compensation.py)及
[遥操初始化](../overlay/src/functions/aimdk_functions/upper_body_teleop_observe.py)。

**操作**

1. 对实验C的姿态做补偿关／开配对，每个条件三次；顺序交替，记录起点HAL目标、反馈、工具与温度等可用状态。
2. 开启补偿的保持窗口放在该侧`blend=1`之后；补偿渐入单独作图。
   松开Grip保持理想目标，补偿仍会随IMU／腰反馈更新，不能把`q_ik`恒定等同于`q_send`恒定。
3. 比较配对姿态的实际`q_ik`差异。首次Grip会重新采样HAL命令，手动复现和相同PICO姿势都不能保证相同理想目标。
   先报告各自姿态和姿态差；无法匹配时仅作探索比较，不据此认定补偿收益。
4. 需要严格配对时，下一步新增最小诊断输入：从保存的理想14轴目标重放静态保持或同一关节序列，
   复用原生遥操执行与日志，分别开关补偿。该入口尚未实现，本轮设计不声称已有确定性执行回放。
5. 逐轴对照无补偿缺口`q_ik-q_fb`、模型`τg/40`、实际`b`和残余`e_ideal`，记录饱和、姿态、IMU／腰输入及源时间差。

**判别**

| 观测 | 解释与下一步 |
| --- | --- |
| 补偿后`e_ideal`及TCP任务误差下降，`e_send`仍大 | 补偿有益；发送目标差值本身不能用于否定收益 |
| 残余方向与无补偿缺口一致，且模型偏置偏小 | 支持偏置不足；检查工具质量遗漏、刚度与重力估计，暂不直接增大参数 |
| 残余反向或误差增大 | 检查过补偿、符号、坐标变换及配对姿态 |
| 偏置达到±12°且仍有缺口 | 饱和限制可能参与，先记录饱和关节和工况 |
| 残余随IMU／腰变化而改变 | 检查重力方向计算和时序，也可能是身体运动影响；需要分层证据 |
| 静态改善、动态改善有限 | 进一步评价滞后、摩擦及当前静态重力模型未描述的动态因素 |

首轮只评估现有算法；工具质量／质心、关节刚度及增益扫描留在严格配对之后。
新增负载试验前核实实际质量、质心原点和实现约定，当前不使用未经测量的负载参数。

## 8. 实验E：实体TCP运动学与浮动基座的后续分离

当前无外部测量设备，本实验等待条件具备。IMU、腰反馈和arm反馈可记录相关性，不能独立确认实体TCP或世界位置。

需要同时取得实体躯干位姿`T_WB`和同一物理TCP的世界位姿`T_WP_ext`，并标定测量标记与模型坐标的固定变换。
采用多姿态、含腕部方向变化的静态测量，先求`T_BP_ext=inv(T_WB)×T_WP_ext`，与`F(q_fb)`比较。
该残差揭示模型、零位、工具变换与结构形变等合成影响；要进一步区分各项，需额外标定与验证姿态。

世界系分析先确定任务定义：

- **随躯干移动的目标：** `T_WP_des(t)=T_WB(t)×T_des(t)`。评价躯干相对跟随能力。
- **固定世界参考目标：** 在参考时刻冻结`T_WB(0)`，使用`T_WP_ref(t)=T_WB(0)×T_des(t)`。
  将实际基座变化与B系任务误差分别传播到世界系，评价身体运动的影响。

当前入口按B系目标控制，固定世界位置保持尚未实现。
仅当独立测量确认B系实体误差小、固定世界参考误差随基座变化增大时，才能将该部分归入浮动基座影响。
当前B系名义FK误差本身不能由世界坐标漂移直接解释；身体运动还可能经动力学间接改变关节跟踪。

## 9. 首轮执行与记录

先完成实验A的离线分解，再按C的无补偿静态基线、D的补偿配对、B的动态片段推进。
静态优先右臂三个姿态各三次；明确主因后左臂复测，最后双臂复测。
每次会话记录实验编号、姿态、实际工具、补偿状态、目标幅度、实际持续时间与现场操作。

### 9.1 公共准备：机器人容器与本地终端

用户同步代码后，在机器人Humble容器的同一Bash终端执行：

```bash
cd /home/agi/aimdk-fsc
bash scripts/build_overlay.sh remote_control aimdk_functions
source /home/agi/aimdk/install/setup.bash
source /home/agi/aimdk-fsc/install/local_setup.bash
ros2 run aimdk_functions upper_body_teleop_observe --help

TRACKING_ARGS=(
  --device-id TestDevice
  --input-config /home/agi/aimdk-fsc/src/remote_control/config/input.json
  --mapping-config /home/agi/aimdk-fsc/src/remote_control/config/mapping.json
  --sdk-library /opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so
  --timeout 10 --input-timeout 0.5 --duration 60 --display-interval 1
  --record-body
)
```

启动采集前由操作者进入URS/RUNNING、启动Service，在PICO开启Head／Controller tracking和Send。
`TRACKING_ARGS`供下文B、C、D复用；每次采集使用新时间戳目录。
命令不选择运动侧，案例名中的left／right仅标记实验：单侧实验只操作该侧Grip，另一侧保持释放。
`--duration 60`从输入接收线程启动起计时，包含首次Grip前等待；等待过久、未形成完整保持窗口时重新采集该轮。

日志下载使用现有“机器人：下载日志”任务。下文离线命令在本地项目根目录的另一个Bash终端执行：

```bash
cd /home/gary/projects/aimdk-fsc
export PYTHONPATH="$PWD/overlay/src/functions${PYTHONPATH:+:$PYTHONPATH}"
TRACKING_ANALYSIS_ROOT="exports/tracking_$(date +%Y%m%d_%H%M%S)"
python3 -m aimdk_functions.upper_body_teleop_analyze --help
```

基本分析只依赖NumPy；命令添加`--plots`后输出PNG，需要本地matplotlib。
每组输出目录必须新建，重复分析时重新设置`TRACKING_ANALYSIS_ROOT`或更换输出后缀。

### 9.2 实验A：已有日志离线分解

本地执行，无需启动机器人或PICO：

```bash
TRACKING_LOG="log/teleop_no_comp_20261103_205907/upper_body_1793711187927434632_50724dcb.jsonl"
python3 -m aimdk_functions.upper_body_teleop_analyze "$TRACKING_LOG" \
  --side both --output "$TRACKING_ANALYSIS_ROOT/A_full" --plots
python3 -m aimdk_functions.upper_body_teleop_analyze "$TRACKING_LOG" \
  --side right --start 5 \
  --output "$TRACKING_ANALYSIS_ROOT/A_right_after5s" --plots
```

核对`summary.json`的metadata、退出原因、IK残差、`task_mm_norm`与`tracking_mm_norm`；
再用CSV中的三项有符号位置分解与`bias_deg`、`send_error_deg`、`ideal_error_deg`解释误差。
第二条命令只截取首条发布5秒后的记录，不自动判定静态或补偿完成；阶段结果另看`phases`。
此历史日志缺新增时间关联，不运行延迟拟合。

### 9.3 实验C：无补偿静态保持

每个姿态与重复轮次单独启动一次。机器人容器执行，依次填写案例名与实际工具说明：

```bash
TRACKING_CASE="C_right_pose1_rep1"
TRACKING_RUN_DIR="/home/agi/aimdk-fsc/log/tracking_${TRACKING_CASE}_$(date +%Y%m%d_%H%M%S)"
read -r -p '实际工具、固定夹爪开度及负载说明：' TRACKING_LOAD
ros2 run aimdk_functions upper_body_teleop_observe "${TRACKING_ARGS[@]}" \
  --xr-output "$TRACKING_RUN_DIR/xr.jsonl" --log-dir "$TRACKING_RUN_DIR" \
  --load-description "$TRACKING_CASE；$TRACKING_LOAD" \
  --no-compensation --execute
```

按第6节到达指定姿态后松开Grip，保持PICO发送至少10秒；保持完成后可Ctrl-C收尾，退出原因应为`interrupted`。
记录到达方向与保持起止时刻。三姿态分别使用`pose1`自然屈肘、`pose2`向前伸出、`pose3`侧向展开，
每个姿态更换`rep1`／`rep2`／`rep3`重跑上段；左侧复测时更换案例名并仅操作左Grip。

下载后，本地填写该轮目录，先分析全程，再依据CSV确定静态末5秒的真实起止时刻：

```bash
read -r -p '本轮本地会话目录（log/tracking_C_...，末尾不加斜杠）：' TRACKING_LOCAL_DIR
TRACKING_LOGS=("$TRACKING_LOCAL_DIR"/upper_body_*.jsonl)
TRACKING_LOG="${TRACKING_LOGS[0]}"
python3 -m aimdk_functions.upper_body_teleop_analyze "$TRACKING_LOG" \
  --side right --output "$TRACKING_ANALYSIS_ROOT/${TRACKING_LOCAL_DIR##*/}_full" --plots

read -r -p '静态末5秒窗口起点（相对首条发布，秒）：' TRACKING_START
read -r -p '静态窗口终点（秒）：' TRACKING_END
python3 -m aimdk_functions.upper_body_teleop_analyze "$TRACKING_LOG" \
  --side right --start "$TRACKING_START" --end "$TRACKING_END" \
  --output "$TRACKING_ANALYSIS_ROOT/${TRACKING_LOCAL_DIR##*/}_static" --plots
```

新采集目录每轮只有一个`upper_body_*.jsonl`；分析前核对数组所选路径与采集终端显示的日志一致。
窗口必须位于松开Grip后的连续保持段；左侧复测将两条命令的`--side right`改为`left`。
核对`feedback_counts`中的IMU／腰计数，静态窗口内`ideal_rad`不变，并查看反馈误差是否仍有趋势。

### 9.4 实验D：补偿开关配对

关闭补偿组使用实验C的数据。为每个姿态再采集对应开启组，机器人容器执行：

```bash
TRACKING_CASE="D_right_pose1_rep1_on"
TRACKING_RUN_DIR="/home/agi/aimdk-fsc/log/tracking_${TRACKING_CASE}_$(date +%Y%m%d_%H%M%S)"
read -r -p '与配对关闭组一致的工具、开度及负载说明：' TRACKING_LOAD
ros2 run aimdk_functions upper_body_teleop_observe "${TRACKING_ARGS[@]}" \
  --xr-output "$TRACKING_RUN_DIR/xr.jsonl" --log-dir "$TRACKING_RUN_DIR" \
  --load-description "$TRACKING_CASE；$TRACKING_LOAD" \
  --compensation --compensation-duration 5 --execute
```

按第7节交替采集关／开条件；本轮对应`C_right_pose1_rep1`，其余姿态、重复和侧别使用相同编号。
到达姿态后松开Grip继续发送；保持窗口需在该侧`blend=1`之后，并具备完整10秒保持。
下载后复用9.3的本地命令，填写本轮D目录与本轮窗口，分别生成`_full`和`_static`结果。
比较配对报告的`ideal_rad.mean`与CSV中的理想目标，再比较`ideal_error_deg`和`task_mm_norm`。
同时核对`bias_deg`、`saturated_publish_samples`与`blend`；目标不匹配时按探索性对照记录。
目前没有相同绝对理想目标的执行重放命令，严格配对入口仍待实现。

### 9.5 实验B：停走、反向与动态滞后

机器人容器执行，先右侧X方向慢速组：

```bash
TRACKING_CASE="B_right_x_slow_rep1"
TRACKING_RUN_DIR="/home/agi/aimdk-fsc/log/tracking_${TRACKING_CASE}_$(date +%Y%m%d_%H%M%S)"
read -r -p '实际工具、固定夹爪开度、路径幅度及负载说明：' TRACKING_LOAD
ros2 run aimdk_functions upper_body_teleop_observe "${TRACKING_ARGS[@]}" \
  --xr-output "$TRACKING_RUN_DIR/xr.jsonl" --log-dir "$TRACKING_RUN_DIR" \
  --load-description "$TRACKING_CASE；$TRACKING_LOAD" \
  --no-compensation --execute
```

按第5节执行保持、正向移动、松开保持、重按反向移动、松开保持。更换`x`为`y`／`z`，
更换`slow`为`fast`记录较快组；实际幅度和速度以日志为准。动作结束后保留保持段再退出。
此处`x/y/z`为期望机器人B系运动方向，第一轮保持工具姿态，案例名不向程序施加轴约束。

下载后，本地先生成无延迟分析结果，查看发送目标与反馈曲线，确定含正反向运动的窗口及延迟搜索上界：

```bash
read -r -p '本轮本地会话目录（log/tracking_B_...）：' TRACKING_LOCAL_DIR
TRACKING_LOGS=("$TRACKING_LOCAL_DIR"/upper_body_*.jsonl)
TRACKING_LOG="${TRACKING_LOGS[0]}"
python3 -m aimdk_functions.upper_body_teleop_analyze "$TRACKING_LOG" \
  --side right --output "$TRACKING_ANALYSIS_ROOT/${TRACKING_LOCAL_DIR##*/}_full" --plots

read -r -p '拟合窗口起点（相对首条发布，秒）：' TRACKING_START
read -r -p '拟合窗口终点（秒）：' TRACKING_END
read -r -p '根据停走响应选定的非负延迟搜索上界（秒，须大于0）：' TRACKING_DELAY_MAX
python3 -m aimdk_functions.upper_body_teleop_analyze "$TRACKING_LOG" \
  --side right --start "$TRACKING_START" --end "$TRACKING_END" \
  --delay-max "$TRACKING_DELAY_MAX" --delay-step 0.002 \
  --output "$TRACKING_ANALYSIS_ROOT/${TRACKING_LOCAL_DIR##*/}_delay" --plots
```

查看`summary.json`中的`sides.right.delay`：`status`、`delay_s`、`at_search_boundary`、
`zero_delay_validation_demeaned_rmse_deg`与`validation_demeaned_rmse_deg`。
搜索结果在上边界时复核所选窗口和上界依据；`unavailable`时按reason检查新日志字段、独立反馈数与命令缺口。
拟合和验证分别使用窗口前2/3与后1/3，选窗时让两个部分均覆盖运动；慢／快组独立分析，不能直接以人工动作差异判断算法收益。

### 9.6 实验E与结果交付

实验E尚无外部测量条件及采集适配，不提供当前不能执行的测量命令。已有B、C、D的IMU／腰记录仅供身体变化相关性分析。
`upper_body_control joint`可辅助重构回归，但其±5°跟踪／到位检查可能提前终止，且TCP入口使用腕原点，
不能直接代替本实验的持续遥操保持记录。既有保护与模式行为保持原契约。

每轮`upper_body_*.jsonl`与`xr.jsonl`保留在对应log目录；分析输出包含`summary.json`、`report.md`、
分侧CSV和启用绘图时的PNG。字段定义见[实验日志与分析说明](../overlay/src/functions/README.md#跟随误差实验日志与离线分析)。

分析结果放入`exports/`，每份报告包含源日志摘要、模型／工具摘要、开关及窗口、原始和延迟对齐后指标、
分轴曲线、配对姿态差、支持的解释与仍不能区分的因素。
首轮完成条件为：IK内部残差可复核；静态与动态误差分开报告；补偿效果有匹配条件或明确的配对局限。
独立测量与确定性目标重放未具备时，分别保留实验E和严格补偿归因为待完成项。
