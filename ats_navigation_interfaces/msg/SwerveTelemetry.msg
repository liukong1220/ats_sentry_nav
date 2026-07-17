# 固定轮序为 [lf, lr, rf, rr]；角度/角速度使用 rad 与 rad/s。
std_msgs/Header header
uint64 sequence

float64[4] drive_rpm
float64[4] wheel_speed_mps
float64[4] steer_angle
float64[4] steer_rate
float64[4] longitudinal_slip_mps
float64[4] lateral_slip_mps

float64 command_vx
float64 command_vy
float64 command_wz
float64 measured_vx
float64 measured_vy
float64 measured_wz

bool[4] drive_speed_saturated
bool[4] drive_acceleration_saturated
bool[4] steer_rate_saturated
uint64 drive_speed_saturation_count
uint64 drive_acceleration_saturation_count
uint64 steer_rate_saturation_count

# 只统计机器人与非地面几何体接触；正常四轮接地不计入。
uint64 contact_violation_count
float64 max_contact_force
