# header.stamp 和 reference_stamp 共同标识经 MINCO snapshot/heartbeat 复核后的候选 reference。
std_msgs/Header header
uint64 goal_id
uint64 localization_epoch
# Echoes PlannerGoal.plan_request_sequence.  Together with goal_id and the
# localization identity this rejects delayed status/reference pairs from a
# prior recovery attempt.
uint64 plan_request_sequence
# MINCO local immutable PlanningMapSnapshot generation, not the ROG generation.
uint64 map_generation
uint64 map_publication_sequence
uint8 STATE_ACCEPTED=0
uint8 STATE_REFERENCE_READY=1
uint8 STATE_FAILED=2
uint8 state
builtin_interfaces/Time reference_stamp
uint8 FAILURE_NONE=0
uint8 FAILURE_INVALID_GOAL=1
uint8 FAILURE_MAP_UNREADY=2
uint8 FAILURE_START_TF=3
uint8 FAILURE_GOAL_TF=4
uint8 FAILURE_START_OR_GOAL_OCCUPIED=5
uint8 FAILURE_NO_PATH=6
uint8 FAILURE_OPTIMIZER=7
uint8 FAILURE_FOOTPRINT=8
uint8 FAILURE_REPAIR=9
uint8 FAILURE_REFERENCE_TF=10
uint8 FAILURE_SNAPSHOT_CHANGED=11
uint8 FAILURE_RUNTIME_UNSAFE=12
uint8 FAILURE_START_ENCLOSED=13
uint8 failure_reason
# Planner-selected yaw policy associated with this candidate reference.
uint8 YAW_AUTHORITY_HOLD_SAFE_STOP=0
uint8 YAW_AUTHORITY_GIMBAL_COMPENSATED=1
uint8 YAW_AUTHORITY_BODY_YAW_FOLLOW=2
uint8 yaw_authority
bool requires_gimbal_lock
