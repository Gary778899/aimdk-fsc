# SigSet.msg
# 信号集合定义

# 整机行为后处理信号
int32 robot_sig

# 运动行为信号
uint8 motion_sig

# 声音行为信号
uint8 sound_sig

# 灯行为信号
uint8 led_sig

# 表情行为信号
uint8 facial_sig

# 预留信号
uint32 reserve

# 功能禁用信号
uint64 funcdisable_sig

