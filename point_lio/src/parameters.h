// #ifndef PARAM_H
// #define PARAM_H
#pragma once
#include <Python.h>
#include <ivox/ivox3d.h>
#include <math.h>
#include <omp.h>
#include <pcl/common/transforms.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <fstream>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "IMU_Processing.h"
#include "preprocess.h"

// #define IVOX_NODE_TYPE_PHC

#ifdef IVOX_NODE_TYPE_PHC
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;
extern IVoxType::Options ivox_options_;
extern int ivox_nearby_type;
extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu;
extern bool space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern int init_map_size;
extern double match_s, satu_acc, satu_gyro;
extern float plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
// extern double cube_len;
extern float DET_RANGE;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov;
extern int pcd_save_interval;
extern std::vector<double> gravity;
extern bool pcd_save_en;
extern bool scan_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern shared_ptr<ImuProcess> p_imu;
extern bool is_first_frame;

extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern double time_diff_lidar_to_imu;
extern double lidar_time_inte, first_imu_time;
extern int orig_odom_freq;
extern double online_refine_time;  //unit: s
extern double time_update_last, time_current, time_predict_last_const, t_last;

extern MeasureGroup Measures;

void readParameters(std::shared_ptr<rclcpp::Node> & n);
/// 启动时打印全部实际生效的参数值。实车与仿真都存在两份候选 YAML
/// （ats_sentry_bringup/params/node_params.yaml 与 point_lio/config/mid360.yaml），
/// 生效值只能靠这条日志确认，禁止靠加载顺序隐式决定。
void logEffectiveParameters(const std::shared_ptr<rclcpp::Node> & n);
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output);