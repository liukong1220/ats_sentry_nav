#include "parameters.h"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu = true, check_satu = true;
bool space_down_sample = true, publish_odometry_without_downsample = false;
int init_map_size = 10;
double match_s = 81, satu_acc, satu_gyro;
float plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5, fov_deg = 180;
// double cube_len = 2000;
float DET_RANGE = 450;
double imu_time_inte = 0.005;
double laser_point_cov = 0.01, acc_norm;
double vel_cov;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov;
int pcd_save_interval;
std::vector<double> gravity;
bool pcd_save_en, extrinsic_est_en = true;
bool scan_pub_en;
shared_ptr<Preprocess> p_pre;
shared_ptr<ImuProcess> p_imu;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
double time_diff_lidar_to_imu = 0.0;

double lidar_time_inte = 0.1, first_imu_time = 0.0;
int orig_odom_freq = 10;
double online_refine_time = 20.0;  //unit: s

MeasureGroup Measures;

void readParameters(std::shared_ptr<rclcpp::Node> & nh)
{
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  try {
    nh->declare_parameter<bool>("prop_at_freq_of_imu", true);
    nh->get_parameter("prop_at_freq_of_imu", prop_at_freq_of_imu);

    nh->declare_parameter<bool>("check_satu", true);
    nh->get_parameter("check_satu", check_satu);

    nh->declare_parameter<int>("init_map_size", 100);
    nh->get_parameter("init_map_size", init_map_size);

    nh->declare_parameter<bool>("space_down_sample", true);
    nh->get_parameter("space_down_sample", space_down_sample);

    nh->declare_parameter<double>("mapping.satu_acc", 3.0);
    nh->get_parameter("mapping.satu_acc", satu_acc);

    nh->declare_parameter<double>("mapping.satu_gyro", 35.0);
    nh->get_parameter("mapping.satu_gyro", satu_gyro);

    nh->declare_parameter<double>("mapping.acc_norm", 1.0);
    nh->get_parameter("mapping.acc_norm", acc_norm);

    nh->declare_parameter<float>("mapping.plane_thr", 0.05f);
    nh->get_parameter("mapping.plane_thr", plane_thr);

    nh->declare_parameter<int>("point_filter_num", 2);
    nh->get_parameter("point_filter_num", p_pre->point_filter_num);

    nh->declare_parameter<std::string>("common.lid_topic", ".livox.lidar");
    nh->get_parameter("common.lid_topic", lid_topic);

    nh->declare_parameter<std::string>("common.imu_topic", ".livox.imu");
    nh->get_parameter("common.imu_topic", imu_topic);

    nh->declare_parameter<double>("common.time_diff_lidar_to_imu", 0.0);
    nh->get_parameter("common.time_diff_lidar_to_imu", time_diff_lidar_to_imu);

    nh->declare_parameter<double>("filter_size_surf", 0.5);
    nh->get_parameter("filter_size_surf", filter_size_surf_min);

    nh->declare_parameter<double>("filter_size_map", 0.5);
    nh->get_parameter("filter_size_map", filter_size_map_min);

    nh->declare_parameter<float>("mapping.det_range", 300.f);
    nh->get_parameter("mapping.det_range", DET_RANGE);

    nh->declare_parameter<double>("mapping.fov_degree", 180);
    nh->get_parameter("mapping.fov_degree", fov_deg);

    nh->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    nh->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);

    nh->declare_parameter<double>("mapping.imu_time_inte", 0.005);
    nh->get_parameter("mapping.imu_time_inte", imu_time_inte);

    nh->declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
    nh->get_parameter("mapping.lidar_meas_cov", laser_point_cov);

    nh->declare_parameter<double>("mapping.vel_cov", 20);
    nh->get_parameter("mapping.vel_cov", vel_cov);

    nh->declare_parameter<double>("mapping.gyr_cov_output", 0.1);
    nh->get_parameter("mapping.gyr_cov_output", gyr_cov_output);

    nh->declare_parameter<double>("mapping.acc_cov_output", 0.1);
    nh->get_parameter("mapping.acc_cov_output", acc_cov_output);

    nh->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    nh->get_parameter("mapping.b_gyr_cov", b_gyr_cov);

    nh->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    nh->get_parameter("mapping.b_acc_cov", b_acc_cov);

    nh->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
    nh->get_parameter("mapping.imu_meas_acc_cov", imu_meas_acc_cov);

    nh->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
    nh->get_parameter("mapping.imu_meas_omg_cov", imu_meas_omg_cov);

    nh->declare_parameter<double>("preprocess.blind", 1.0);
    nh->get_parameter("preprocess.blind", p_pre->blind);

    nh->declare_parameter<int>("preprocess.scan_line", 16);
    nh->get_parameter("preprocess.scan_line", p_pre->N_SCANS);

    nh->declare_parameter<double>("mapping.match_s", 81);
    nh->get_parameter("mapping.match_s", match_s);

    nh->declare_parameter<std::vector<double>>("mapping.gravity", std::vector<double>());
    nh->get_parameter("mapping.gravity", gravity);

    nh->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
    nh->get_parameter("mapping.extrinsic_T", extrinT);

    nh->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());
    nh->get_parameter("mapping.extrinsic_R", extrinR);

    nh->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
    nh->get_parameter(
      "odometry.publish_odometry_without_downsample", publish_odometry_without_downsample);

    nh->declare_parameter<bool>("publish.scan_publish_en", true);
    nh->get_parameter("publish.scan_publish_en", scan_pub_en);

    nh->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    nh->get_parameter("pcd_save.pcd_save_en", pcd_save_en);

    nh->declare_parameter<int>("pcd_save.interval", -1);
    nh->get_parameter("pcd_save.interval", pcd_save_interval);

    nh->declare_parameter<double>("mapping.lidar_time_inte", 0.1);
    nh->get_parameter("mapping.lidar_time_inte", lidar_time_inte);

    nh->declare_parameter<float>("mapping.ivox_grid_resolution", 0.2);
    nh->get_parameter("mapping.ivox_grid_resolution", ivox_options_.resolution_);

    nh->declare_parameter<int>("ivox_nearby_type", 18);
    nh->get_parameter("ivox_nearby_type", ivox_nearby_type);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_ERROR(nh->get_logger(), "Parameter type exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(nh->get_logger(), "Exception: %s", e.what());
  }

  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    // LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }
  if (gravity.size() >= 3) {
    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
  } else {
    RCLCPP_ERROR(
      nh->get_logger(),
      "[point_lio 生效参数] mapping.gravity 只有 %zu 个元素（需要 3 个）：拒绝越界读取，"
      "保留 IMU 处理器内的默认重力。这属于参数源缺失，必须按停止条件处理。",
      gravity.size());
  }

  logEffectiveParameters(nh);
}

void logEffectiveParameters(const std::shared_ptr<rclcpp::Node> & nh)
{
  // 实车只有一份权威参数源（ats_sentry_bringup/params/node_params.yaml 的
  // point_lio 段），但 point_lio 包内还留着一份 config/mid360.yaml，两份数值不同。
  // 这里在启动时把「真正生效的值」原样打出来，禁止靠 YAML 加载顺序或
  // 「哪份文件先被 include」来隐式决定。日志里出现的数值就是运行时用的数值。
  const auto join = [](const std::vector<double> & values) {
    if (values.empty()) {
      return std::string("[]");
    }
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
      if (i != 0) {
        oss << ", ";
      }
      oss << std::setprecision(9) << values[i];
    }
    oss << "]";
    return oss.str();
  };

  bool use_sim_time = false;
  nh->get_parameter("use_sim_time", use_sim_time);

  RCLCPP_INFO(
    nh->get_logger(),
    "[point_lio 生效参数] use_sim_time=%s lid_topic=%s imu_topic=%s "
    "point_filter_num=%d space_down_sample=%s",
    use_sim_time ? "true" : "false", lid_topic.c_str(), imu_topic.c_str(), p_pre->point_filter_num,
    space_down_sample ? "true" : "false");
  RCLCPP_INFO(
    nh->get_logger(),
    "[point_lio 生效参数] filter_size_surf=%.6f filter_size_map=%.6f "
    "ivox_nearby_type=%d ivox_grid_resolution=%.6f init_map_size=%d",
    filter_size_surf_min, filter_size_map_min, ivox_nearby_type,
    static_cast<double>(ivox_options_.resolution_), init_map_size);
  RCLCPP_INFO(
    nh->get_logger(),
    "[point_lio 生效参数] lidar_time_inte=%.6f imu_time_inte=%.6f time_diff_lidar_to_imu=%.6f",
    lidar_time_inte, imu_time_inte, time_diff_lidar_to_imu);
  RCLCPP_INFO(
    nh->get_logger(),
    "[point_lio 生效参数] preprocess: scan_line=%d blind=%.6f det_range=%.3f fov_deg=%.3f",
    p_pre->N_SCANS, p_pre->blind, static_cast<double>(DET_RANGE), fov_deg);
  RCLCPP_INFO(
    nh->get_logger(),
    "[point_lio 生效参数] mapping: satu_acc=%.6f satu_gyro=%.6f acc_norm=%.6f "
    "lidar_meas_cov=%.6f plane_thr=%.6f match_s=%.3f extrinsic_est_en=%s",
    satu_acc, satu_gyro, acc_norm, laser_point_cov, static_cast<double>(plane_thr), match_s,
    extrinsic_est_en ? "true" : "false");
  // 外参是雷达安装位姿的唯一数值来源之一：全零意味着「没有实测外参」，
  // 不是「外参为零」。这条日志必须能在实车日志里被直接引用为证据。
  RCLCPP_INFO(
    nh->get_logger(), "[point_lio 生效参数] extrinsic_T=%s extrinsic_R=%s", join(extrinT).c_str(),
    join(extrinR).c_str());
  RCLCPP_INFO(nh->get_logger(), "[point_lio 生效参数] gravity=%s", join(gravity).c_str());
  RCLCPP_INFO(
    nh->get_logger(),
    "[point_lio 生效参数] publish: scan_publish_en=%s publish_odometry_without_downsample=%s",
    scan_pub_en ? "true" : "false", publish_odometry_without_downsample ? "true" : "false");
  if (std::all_of(extrinT.begin(), extrinT.end(), [](double v) { return v == 0.0; })) {
    RCLCPP_WARN(
      nh->get_logger(),
      "[point_lio 生效参数] extrinsic_T 全零：IMU->LiDAR 外参未实测。抬轮 HIL 之外不得据此"
      "推断定位精度。");
  }
}

void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
  P_init_output = MD(30, 30)::Identity() * 0.01;
  P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
  // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
  P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
}