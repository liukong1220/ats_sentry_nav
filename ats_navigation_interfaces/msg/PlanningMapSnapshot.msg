# P4 atomic adapter publication. This replaces the cross-topic association of a
# planning grid and PlanningMapStatus only after every consumer has migrated.
# header.stamp is adapter publication time in the ROS clock; source_stamp is the
# observation/map-source time. Lease age remains a consumer-local steady-clock check.
std_msgs/Header header
builtin_interfaces/Time source_stamp
bool ready
bool unknown_is_obstacle
int8 occupied_value_threshold
uint64 localization_epoch
uint64 source_generation
uint64 publication_sequence
# Geometry is equivalent to nav_msgs/OccupancyGrid.info and retains origin yaw.
nav_msgs/MapMetaData info
# -1 is unknown. Every ready payload has width * height cells.
int8[] occupancy
# Per-cell numeric RC-ESDF values in metres. Unknown cells use NaN in all three
# numeric arrays; known free values are non-negative and occupied values are non-positive.
float32[] signed_distance_m
float32[] gradient_x
float32[] gradient_y
