# P4 immutable MINCO candidate. The producer publishes one self-consistent
# candidate rather than status/reference pairs on independent DDS topics.
std_msgs/Header header
uint8 STATE_REJECTED=0
uint8 STATE_READY=1
uint8 state
# Nonzero and process-lifetime stable. candidate_sequence is strictly increasing
# inside an incarnation; a new incarnation starts by publishing STATE_REJECTED.
uint64 planner_incarnation
uint64 candidate_sequence
uint64 goal_id
uint64 localization_epoch
# These three generations remain distinct by design.
uint64 map_snapshot_generation
uint64 map_source_generation
uint64 map_publication_sequence
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
uint8 failure_reason
uint8 YAW_AUTHORITY_HOLD_SAFE_STOP=0
uint8 YAW_AUTHORITY_GIMBAL_COMPENSATED=1
uint8 YAW_AUTHORITY_BODY_YAW_FOLLOW=2
uint8 yaw_authority
bool requires_gimbal_lock
bool footprint_safe
bool dynamics_feasible
uint32 footprint_collision_samples
float32 minimum_clearance_m
# Relative to receipt time; consumers still use a local steady-clock deadline.
builtin_interfaces/Duration lease_duration
# SHA-256 of the immutable candidate identity and reference content, excluding
# heartbeat publication timestamps and this field itself. All-zero is invalid for STATE_READY.
uint8[32] content_digest
nav_msgs/Path raw_path
nav_msgs/Path reference
