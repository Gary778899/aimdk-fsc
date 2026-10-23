# Header 包含时间辍, trace_id、domain
MessageHeader header

# 表情文件的路径
string e_path

# 当前一组表情的路径列表
string[] e_path_list

# 表情id，可以为空
uint8 e_id

# 播放模式，1-仅播放一次，2-循环播放
uint8 mode

# 默认表情优先级为 0, 数值越大优先级越高
int32 priority

# 当前状态枚举：idle | start（仅广播一次）| running | finished（仅广播一次）| stopped（仅广播一次）
uint8 STATUS_IDLE=0
uint8 STATUS_START=1
uint8 STATUS_RUNNING=2
uint8 STATUS_FINISHED=3
uint8 STATUS_STOPPED=4

uint8 status

# 剩余时长，单位：毫秒；如果重复，会将剩余重复时间计算在内
float64 time_to_end_ms

# 当前活跃的显示层类型：0=idle, 1=video, 2=svga, 3=rgb, 4=image, 5=qr, 6=text
uint8 display_type

# 关联触发本次状态变更的 RPC 调用 ID，用于调用方确认执行结果
string trace_id

# 执行失败原因；0 表示无错误
int32 error_code
