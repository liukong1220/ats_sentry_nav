# header.stamp 和 reference_stamp 共同标识经 MINCO snapshot/heartbeat 复核后的候选 reference。
std_msgs/Header header
uint64 goal_id
uint64 localization_epoch
uint64 map_generation
uint8 STATE_ACCEPTED=0
uint8 STATE_REFERENCE_READY=1
uint8 STATE_FAILED=2
uint8 state
builtin_interfaces/Time reference_stamp
string reason
