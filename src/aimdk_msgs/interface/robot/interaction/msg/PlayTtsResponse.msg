# PlayMediaFileRequest 同样复用此 rsp，file_name 放在 text 中
string text                                

TtsPriorityLevel priority_level

# 加权参数(0-10)，最小单位为 1
uint32 priority_weight   

# 标识调用方来源
string domain

string trace_id

# 是否成功播放
bool is_success

# 错误信息
string error_message

# 当前文本预计耗时（单位：毫秒）
uint32 estimated_duration

# 错误码
uint8 error_code

# ========== 错误码枚举 ==========
uint8 ERROR_NONE                  = 0  # 无错误，对应 is_success = true
uint8 ERROR_HIGHER_PRIORITY_BUSY  = 1  # 当前正在处理更高优先级的播报，本次请求被让位
uint8 ERROR_NETWORK_DISCONNECTED  = 2  # 网络断开
uint8 ERROR_UNKNOWN               = 3  # 未知错误