# header.stamp 是目标管理器提交该请求的时间，goal_id 只在本次进程生命周期内单调递增。
std_msgs/Header header
uint64 goal_id
# 0 保留给无定位融合的 P3 legacy 路径；P4 请求必须与当前定位 epoch 一致。
uint64 localization_epoch
geometry_msgs/PoseStamped goal_pose
