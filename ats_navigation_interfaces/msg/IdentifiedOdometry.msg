# Odometry and source process identity must travel in the same DDS sample.
# instance_id is Point-LIO startup steady-clock nanoseconds, nonzero. Ordering is
# valid only when Point-LIO and fusion share one machine and the same boot.
std_msgs/Header header
uint64 instance_id
nav_msgs/Odometry odometry
