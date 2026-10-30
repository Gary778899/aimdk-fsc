# -----------------------------------------------------------------------------------
# 机器人运控状态 - 伺服控制运行状态
# -----------------------------------------------------------------------------------

# --------- squat ---------
bool squat_enable                             # 是否处于下蹲状态

# squat range: 0.0 ~ -0.5
float64 squat_offset                    # 下蹲高度偏置         

uint32 squat_limit                      # 下蹲上下限位

uint32 BEFORE_LIMIT = 0                 # 未到限位

uint32 LOWER_LIMIT  = 1                 # 下蹲到下限

uint32 UPPER_LIMIT  = 2                 # 下蹲到上限

# --------- head ---------
# yaw range: -0.38 ~ 0.38
float64 head_yaw_offset

# pitch range: -0.35 ~ 0.35
float64 head_pitch_offset

# --------- waist ---------
# yaw range: -3.43 ~ 2.382
float64 waist_yaw_offset

# pitch range: -0.314 ~ 0.314
float64 waist_pitch_offset

# roll range: -0.488 ~ 0.488
float64 waist_roll_offset
