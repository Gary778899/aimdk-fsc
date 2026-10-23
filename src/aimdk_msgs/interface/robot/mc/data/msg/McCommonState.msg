# -----------------------------------------------------------------------------------
# 机器人运控状态
# -----------------------------------------------------------------------------------

MessageHeader header

# ------------ 输入源信息 ------------ #
McInputSource input_source

# ----------- Action 信息 ----------- #
McActionInfo action_info

# --------- 机器运行模型信息 ---------- #
McRuntimeModel runtime_model

# --------- 动作运行信息 ---------- #
McMotionStatus motion_status

# --------- 伺服控制运行信息 ---------- #
McServoStatus servo_status

# --------- 有限状态机状态 ---------- #
McFsmState fsm_state

# --------- 机器人姿态状态 ---------- #
McBodyPoseStatus body_status

# --------- 走跑速度状态 ---------- #
McLocomotionSpeedStatus speed_status
