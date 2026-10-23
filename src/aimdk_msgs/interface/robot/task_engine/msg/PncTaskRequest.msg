# PncTaskRequest.msg

# ========== 枚举定义 TASK_TYPE ==========
# !!!! 增加枚举请在末尾增加，千万不要中间插入, 不要调换已有的顺序!!!!
uint8 TASK_TYPE_NONE                          = 0
uint8 TASK_TYPE_PLANNING_NAVI_TO_GOAL         = 1
uint8 TASK_TYPE_PLANNING_NAVI_TO_POSE_2D      = 2
uint8 TASK_TYPE_LINEAR_NAVI_TO_RELATIVE       = 3
uint8 TASK_TYPE_LINEAR_NAVI_TO_GOAL           = 4
uint8 TASK_TYPE_LINEAR_NAVI_TO_POSE_2D        = 5
uint8 TASK_TYPE_DIRECT_NAVI_TO_RELATIVE_GOAL  = 6
uint8 TASK_TYPE_DIRECT_NAVI_TO_RELATIVE       = 7
uint8 TASK_TYPE_SPIN_TURN_AND_MOVE_FORWARD    = 8
uint8 TASK_TYPE_SPIN_TURN                     = 9
uint8 TASK_TYPE_MOVE_FORWARD                  = 10
uint8 TASK_TYPE_PRECISE_NAVI_TO_GOAL          = 11
uint8 TASK_TYPE_NO_MAP_FOLLOW                 = 12
uint8 TASK_TYPE_AUTO_EXPLORE                  = 13
uint8 TASK_TYPE_VOICE_CONTROL                 = 14
uint8 TASK_TYPE_RELATIVE_TARGET_TRACKING      = 15
uint8 TASK_TYPE_PLANNING_NAVI_WITH_RL         = 16
uint8 TASK_TYPE_SWARM_CONTROL_NAVI_MODE         = 17

# ========== 枚举定义 TASK_REQUEST =========
# !!!! 增加枚举请在末尾增加，千万不要中间插入, 不要调换已有的顺序!!!!
uint8 TASK_REQUEST_NONE  = 0
uint8 TASK_REQUEST_START = 1
uint8 TASK_REQUEST_STOP  = 2
uint8 TASK_REQUEST_PAUSE = 3
uint8 TASK_REQUEST_RESUME = 4
uint8 TASK_REQUEST_STATUS_INQUIRY = 5

# ========== 枚举定义 PNC_MODE ===============
uint8 NORMAL        = 0
uint8 NO_PERCEPTION = 1

MessageHeader header

uint8 task_type                             # 枚举定义 TASK_TYPE
uint8 task_request                          # 枚举定义 TASK_REQUEST 
uint8 pnc_mode                              # 枚举定义 PNC_MODE
uint64 task_id                              # 任务ID：区分是导览路线，设定的任务点信息等
uint64 map_id                               # 地图ID：区分当前使用地图
geometry_msgs/PoseStamped target_pose       # 目标位姿                         # 枚举定义 TASK_STATUS
geometry_msgs/Pose2D[] reference_line       # 指引线仅考虑<x,y>, target_pose设置为目标姿态(yaw)
float64 target_pose_radius                  # 目标范围半径, 当 target_pose 被占据时，用于判断该次导航的目标点是否可以到达
float32 max_forward_speed                   # 最大运行速度

geometry_msgs/Pose2D[] trajectory_pos       # 群控模式，轨迹的路点位置
geometry_msgs/Twist[] trajectory_vel  # 群控模式，轨迹的路点速度vx vy wz的填入
uint64 play_timestamp # orin板子上启动机器人行走行为树的系统时间戳
uint64 time_duration  # 群控模式下行走轨迹的时长

# 64 byte 保留字段
char[64] reserve_info