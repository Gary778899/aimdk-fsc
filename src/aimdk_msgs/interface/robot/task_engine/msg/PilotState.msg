# 定义领航任务状态
uint8 TM_PILOT_STATE_NAVI_TO_TARGET	 = 0
uint8 TM_PILOT_STATE_TARGET_ACTIONS  = 1
uint8 TM_PILOT_STATE_NAVI_RETURN     = 2
uint8 TM_PILOT_STATE_STANDBY_ACTIONS = 3

# 目标点别名：若导航至POI，即POI别名； 若导航至讲解点，即讲解点别名
# 目标点ID：若导航至POI，即POI ID(创建POI时生成)； 若导航至讲解点，即讲解ID
string target_name 
uint64 target_id

# 仅在导航至讲解任务创建的讲解点时有效
# 讲解任务别名：用户取的任务别名
# 讲解任务ID
string intro_task_name 
uint64 intro_task_id

# 地图别名：用户取的任务别名. 若当前任务依赖地图时字段有效
# 地图ID：全局唯一的id, 创建地图时生成:
string map_name
uint64 map_id

# 领航任务状态
uint8 pilot_state

