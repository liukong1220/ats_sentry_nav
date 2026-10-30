#pragma once
#include <math.h>

#include <cmath>
// #include <deque>
// #include <mutex>
// #include <thread>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
// #include <so3_math.h>
#include <Eigen/Eigen>
// #include "Estimator.h"
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/// *************Preconfiguration

#define MAX_INI_COUNT (100)
const bool time_list(PointType & x, PointType & y);  // {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  // 去畸变点云直接指向 meas.lidar（不再深拷贝）；调用方在下一次 sync_packages 之前只读使用
  void Process(const MeasureGroup & meas, PointCloudXYZI::Ptr & pcl_un_);
  void Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot);

  MD(12, 12) state_cov = MD(12, 12)::Identity();
  V3D gravity_;
  V3D mean_acc;
  bool imu_need_init_ = true;
  bool after_imu_init_ = false;
  bool b_first_frame_ = true;
  double time_last_scan = 0.0;

private:
  void IMU_init(const MeasureGroup & meas, int & N);
  V3D mean_gyr;
  int init_iter_num = 1;
  rclcpp::Logger logger;
};
