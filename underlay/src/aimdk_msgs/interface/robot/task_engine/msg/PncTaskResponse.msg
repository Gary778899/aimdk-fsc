# PncTaskResponse.msg

# ========== 枚举定义 TASK_TYPE ==========
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

# ========== 枚举定义 TASK_STATUS =========
# refs: https://agirobot.feishu.cn/wiki/MY9mwPTCVi19Q4kfCQtcAv69nub?larkTabName=space
uint8 TASK_STATUS_UNKONW  = 0
uint8 TASK_STATUS_RUNNING = 1
uint8 TASK_STATUS_FAILED  = 2
uint8 TASK_STATUS_PAUSED  = 3
uint8 TASK_STATUS_FINISHED = 4
uint8 TASK_STATUS_UNKNOW_BLOCK = 10
uint8 TASK_STATUS_SPIN_BLOCK = 11
uint8 TASK_STATUS_TRAJ_COLLISION_BLOCK = 12
uint8 TASK_STATUS_CONTOLLER_COLLISION_BLOCK = 13
uint8 TASK_STATUS_SPIN_BLOCK_WITH_HUMAN = 14 
uint8 TASK_STATUS_TRAJ_COLLISION_BLOCK_WITH_HUMAN = 15
uint8 TASK_STATUS_CONTOLLER_COLLISION_BLOCK_WITH_HUMAN = 16
uint8 TASK_STATUS_LOCAL_OPTIMAL_FAILED = 17
uint8 TASK_STATUS_GOBAL_PATH_BLOCK = 18
uint8 TASK_STATUS_RECOVERY_GLOBAL = 20
uint8 TASK_STATUS_RECOVERY_LOCAL = 21
uint8 TASK_STATUS_RECOVERY_TIMEOUT = 22
uint8 TASK_STATUS_EXPLORE_TO_RECOVERY = 23
uint8 TASK_STATUS_EXPLORE_SUCCESS = 24
uint8 TASK_STATUS_EXPLORE_FAILED = 25
uint8 TASK_STATUS_GOAL_BLOCK_CONTINUE = 30
uint8 TASK_STATUS_GOAL_BLOCK_ABORT = 31


MessageHeader header

uint8 task_type                             # 枚举定义 TASK_TYPE
uint64 task_id                              # 同 header.sequence, 描述唯一的某一任务指令(在同一任务的TASK_STATUS中，task_id相同)
uint64 map_id                               # 地图ID：区分当前使用地图
uint8 task_status                           # 枚举定义 TASK_STATUS

# 64 byte 保留字段
char[64] reserve_info