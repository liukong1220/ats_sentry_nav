# header.stamp 是参与配准的最后一帧扫描时间，header.frame_id 是 map_frame。
# pose 表示观测时刻的 map -> child_frame_id 全局位姿；融合节点必须结合
# 同一时刻的 odom -> child_frame_id 历史位姿计算 map -> odom。
std_msgs/Header header
string child_frame_id
uint64 sequence

# 协方差顺序遵循 geometry_msgs/PoseWithCovariance 的 [x,y,z,rx,ry,rz]。
# small_gicp 使用最终信息矩阵逆与残差尺度给出近似值，不代表已完成实车统计标定。
geometry_msgs/PoseWithCovariance pose

uint8 STATUS_ACCEPTED=0
uint8 STATUS_REJECTED=1
uint8 STATUS_NO_ODOM=2
uint8 STATUS_INVALID=3
uint8 STATUS_PENDING_CONFIRMATION=4
uint8 status
bool accepted

uint32 inlier_count
uint32 source_points
# small_gicp 最终目标值除以内点数；不是米制 RMSE。
float64 registration_error
float64 quality
string message
