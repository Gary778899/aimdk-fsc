# 定义固定路线讲解任务状态
uint8 TM_FIX_LINE_INTRO_STATE_NAVI_TO_NEXT_SITE	  = 0
uint8 TM_FIX_LINE_INTRO_STATE_SITE_ACTIONS        = 1
uint8 TM_FIX_LINE_INTRO_STATE_NAVI_RETURN         = 2
uint8 TM_FIX_LINE_INTRO_STATE_NAVI_TO_TARGET_POI  = 3
uint8 TM_FIX_LINE_INTRO_STATE_POI_ACTIONS         = 4
uint8 TM_FIX_LINE_INTRO_STATE_STANDBY_ACTIONS     = 5

# 用户预设置任务的 id, 若未设置过 set_task_id = 0
# 用户预设置任务的别名, 若未设置过 set_task_name = ""
uint64 set_task_id                         
string set_task_name                        

# 任务别名：用户取的任务别名
# 任务ID：全局唯一的id, 创建任务时生成(目前只有创建的导览任务):
string task_name 
uint64 task_id


# 地图别名：用户取的任务别名. 若当前任务依赖地图时字段有效
# 地图ID：全局唯一的id, 创建地图时生成:
string map_name
uint64 map_id

# 剩余讲解点数量，若当前正在讲解，不包含当前讲解点
uint32 rest_site_number 

# 固定路线讲解任务状态
uint8 fix_line_intro_state

# 当前目标点位 id
# 若[TM_FIX_LINE_INTRO_STATE_NAVI_TO_TARGET_POI], 当前插入POI任务-导航过程：id 为该 poi id
# 若[TM_FIX_LINE_INTRO_POI_ACTIONS], 当前插入POI任务-POI动作过程：id 为该 poi id
# 若[TM_FIX_LINE_INTRO_STATE_NAVI_TO_NEXT_SITE], 当前讲解任务-导航过程：id 为下一个 讲解点 id
# 若[TM_FIX_LINE_INTRO_STATE_SITE_ACTIONS], 当前讲解任务-动作过程：id 为当前 讲解点 id
# 若[TM_FIX_LINE_INTRO_STATE_NAVI_RETURN], 在返回待机点过程：id 无效（忽略该字段）
uint64 cur_target_id