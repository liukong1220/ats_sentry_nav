# header.stamp 是目标管理器提交该请求的时间，goal_id 只在本次进程生命周期内单调递增。
std_msgs/Header header
uint64 goal_id
# 0 保留给无定位融合的 P3 legacy 路径；P4 请求必须与当前定位 epoch 一致。
uint64 localization_epoch
# Strictly increasing inside one goal_id.  A Goal Manager recovery must issue a
# new value, so a delayed candidate from an earlier request cannot be retimed
# and re-authorize motion after an emergency stop.
uint64 plan_request_sequence
# Goal Manager snapshots the adapter publication sequence before dispatch.  It is
# distinct from ROG source and MINCO local snapshot generations.
uint64 map_publication_sequence
geometry_msgs/PoseStamped goal_pose
# Frozen at dispatch with the same localization epoch as goal_id / map snapshot.
# start_pose.header.stamp is the localization sample used for this request.
geometry_msgs/PoseStamped start_pose
# World-frame twist at that localization sample. linear.x/y are map/odom world axes.
geometry_msgs/Twist start_twist
