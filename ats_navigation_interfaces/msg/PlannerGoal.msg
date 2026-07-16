# header.stamp 是目标管理器提交该请求的时间，goal_id 只在本次进程生命周期内单调递增。
std_msgs/Header header
uint64 goal_id
geometry_msgs/PoseStamped goal_pose
