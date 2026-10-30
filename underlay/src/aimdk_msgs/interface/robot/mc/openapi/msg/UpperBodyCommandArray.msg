# 上半身（头 + 双臂 + 手）关节/目标指令，供运动控制或下游分解为 HAL 关节命令。
# 与 McBodyPose（躯干/站姿）正交：本消息侧重头、手臂与末端手的目标。

# 消息头（时间戳、坐标系、序列号等）
MessageHeader header

# 指令来源标识，便于仲裁与调试（例如 teleop、planner、replay 等）
string source

# 手部子模式，与 hand_pos 的语义一致，取值见下方 HAND_* 常量
uint8 hand_sub_mode

# 头部关节目标位置，固定长度 2；元素顺序与具体机型 neck 定义一致（常见为两自由度）
# 单位：rad
float64[2] head_pos

# 双臂关节目标位置，固定长度 14：通常按「左臂 7 + 右臂 7」与机型臂链顺序一致
# 单位：rad
float64[14] arm_pos

# 手部目标：长度随 hand_sub_mode 与手型变化（夹爪开合度、灵巧手关节、手势参数等）
# 单位：依子模式约定（多为 rad 或归一化开合量）
float64[] hand_pos

# --- hand_sub_mode 取值（与 HAL/手部驱动约定对齐）---
uint8 HAND_CLAW_OPEN_CLOSE=1
uint8 HAND_DEXTEROUS_JOINT=2
uint8 HAND_DEXTEROUS_GESTURE=3
