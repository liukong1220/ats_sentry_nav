# header.stamp 是融合节点发布健康状态的时间，header.frame_id 是 map_frame。
std_msgs/Header header

uint8 STATE_UNINITIALIZED=0
uint8 STATE_TRACKING=1
uint8 STATE_RELOCALIZING=2
uint8 STATE_DEGRADED=3
uint8 STATE_LOST=4
uint8 state

uint64 epoch
uint64 observation_sequence
builtin_interfaces/Time last_observation_stamp
# 以下 silence 使用 steady clock 计算，不受 /clock 暂停或回拨影响。
float64 odometry_silence_sec
float64 observation_silence_sec
# observation_age_sec 使用 ROS/sim clock 表示测量时间年龄，仅用于诊断。
float64 observation_age_sec
float64 last_correction_translation
float64 last_correction_yaw
uint32 consecutive_rejections
string message
