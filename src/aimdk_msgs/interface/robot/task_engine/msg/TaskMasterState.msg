# 定义(顶层)主状态
uint8 TM_MAIN_STATE_IDLE	            = 0
uint8 TM_MAIN_STATE_RELOCALIZATION    = 1
uint8 TM_MAIN_STATE_MAPPING           = 2
uint8 TM_MAIN_STATE_FIX_LINE_INTRO    = 3
uint8 TM_MAIN_STATE_RECORDING_PATH    = 4
uint8 TM_MAIN_STATE_PILOT             = 5

# 定义(顶层)子状态
uint8 TM_SUB_STATE_NONE               = 0
uint8 TM_SUB_STATE_RESUME             = 1  # 长程任务：运行中
uint8 TM_SUB_STATE_PAUSE              = 2  # 长程任务：暂停中

# 当前主状态
uint8 main_state

# 当前子状态
uint8 sub_state

# 定义SLAM状态
uint8 SLAM_STATE_IDLE	             = 0
uint8 SLAM_STATE_RELOCALIZATION    = 1
uint8 SLAM_STATE_MAPPING           = 2
uint8 SLAM_STATE_LOCALIZATION      = 3

# 当前SLAM状态
uint8 slam_state

# 当前定位地图信息：
# [slam_state]=[SLAM_STATE_LOCALIZATION] 时字段有效
uint64 loc_map_id
string loc_map_name

# 任意状态，用户预设置任务相关字段，总是有效：
# [main_state]=[TM_MAIN_STATE_FIX_LINE_INTRO]时，其他相关字段有效
FixLineIntroState fix_line_intro_state

# [main_state]=[TM_MAIN_STATE_PILOT]时字段有效：
PilotState pilot_state

# 保留信息
string reserve_info

