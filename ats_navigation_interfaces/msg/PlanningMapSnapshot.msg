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
# Per-cell numeric RC-ESDF values in metres, finite in every cell of a ready payload.
# A cell is blocked when occupancy >= occupied_value_threshold, or occupancy == -1 while
# unknown_is_obstacle holds; blocked values are non-positive and all others non-negative.
# Cells outside the grid count as blocked, so a free edge cell is at most one cell from
# the nearest blocked centre. Not-ready payloads may carry NaN for audit only.
float32[] signed_distance_m
float32[] gradient_x
float32[] gradient_y
