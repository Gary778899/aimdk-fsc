# 历史事件记录 ID
uint64 id

# 诊断码
uint64 diag_code

# 告警码
string alert_code

# 告警部位，当前预留
string alert_location

# 首次发生时间戳，单位为纳秒
uint64 first_timestamp

# 最近发生时间戳，单位为纳秒
uint64 last_timestamp

# 恢复时间戳，单位为纳秒，0 表示未恢复
uint64 recover_timestamp

# 持续时长，单位为纳秒
uint64 duration_ns

# 是否已恢复
bool recovered
