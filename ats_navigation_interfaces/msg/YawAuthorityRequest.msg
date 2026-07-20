# Goal Manager requests a protected gimbal/yaw mode transition with this message.
std_msgs/Header header
uint64 request_sequence
uint64 goal_id
uint64 localization_epoch
uint64 map_generation
uint64 map_publication_sequence
uint8 YAW_AUTHORITY_HOLD_SAFE_STOP=0
uint8 YAW_AUTHORITY_GIMBAL_COMPENSATED=1
uint8 YAW_AUTHORITY_BODY_YAW_FOLLOW=2
uint8 yaw_authority
bool require_gimbal_lock
