# 起始时间戳，单位为纳秒，0 表示不限制
uint64 start_time

# 结束时间戳，单位为纳秒，0 表示不限制
uint64 end_time

# 按诊断码过滤
uint64[] diag_codes

# 按告警码过滤
string[] alert_codes

# 按告警部位过滤
string[] alert_locations

# 状态过滤
uint8 state

# 是否按时间倒序
bool order_desc

# 分页偏移
uint32 offset

# 分页条数
uint32 limit

uint8 STATE_ALL = 0
uint8 STATE_ACTIVE = 1
uint8 STATE_RECOVERED = 2
