# Authoritative gimbal feedback.  A status is valid only for its request_sequence.
std_msgs/Header header
uint64 sequence
uint64 request_sequence
uint8 YAW_AUTHORITY_HOLD_SAFE_STOP=0
uint8 YAW_AUTHORITY_GIMBAL_COMPENSATED=1
uint8 YAW_AUTHORITY_BODY_YAW_FOLLOW=2
# 小陀螺：云台补偿，底盘按参考 yaw_rate 边平移边自旋（见 minco_planner 的 SpinPolicy）。
uint8 YAW_AUTHORITY_SPIN=3
uint8 yaw_authority
bool locked
bool tf_healthy
# All values are yaw angles in radians in their documented frames.
float64 gimbal_yaw
float64 body_yaw
float64 fake_yaw
