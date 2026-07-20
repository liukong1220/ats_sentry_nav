# This is the sole execution authorization consumed by the MPC.  A DDS sample
# atomically carries the authorization mode, identity, versions, and reference.
std_msgs/Header header
uint8 MODE_STOP=0
uint8 MODE_EXECUTE=1
uint8 mode
# Strictly monotonic per Goal Manager process.  Consumers reject stale commands.
uint64 command_sequence
uint64 goal_id
uint64 localization_epoch
# MINCO local immutable PlanningMapSnapshot generation.
uint64 map_generation
# Adapter PlanningMapStatus publication sequence.
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
# The yaw authority is part of the execution lease, never a separate Bool.
uint8 YAW_AUTHORITY_HOLD_SAFE_STOP=0
uint8 YAW_AUTHORITY_GIMBAL_COMPENSATED=1
uint8 YAW_AUTHORITY_BODY_YAW_FOLLOW=2
uint8 yaw_authority
bool requires_gimbal_lock
# The exact protected mode request acknowledged for this EXECUTE reference.
uint64 gimbal_request_sequence
# Monotonic feedback sequence that Goal Manager validated after that request.
uint64 gimbal_feedback_sequence
nav_msgs/Path reference
