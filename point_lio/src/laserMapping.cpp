// #include <so3_math.h>
#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <random>

#include <nav_msgs/msg/odometry.hpp>
#include <ats_navigation_interfaces/msg/identified_odometry.hpp>

#include "li_initialization.h"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

bool init_map = false, flg_first_scan = true;

bool flg_exit = false;

//surf feature in map
// 去畸变点云：ImuProcess::Process 之后与 Measures.lidar 指向同一块点云（不再深拷贝），只读使用
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

nav_msgs::msg::Odometry odomAftMapped;

auto LOGGER = rclcpp::get_logger("laserMapping");

void SigHandle(int sig)
{
  flg_exit = true;
  RCLCPP_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

void MapIncremental()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) *
        filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (
          fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);
}

void publish_init_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  int size_init_map = init_feats_world->size();

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_down_world, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes->publish(laserCloudmsg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (pcd_save_en) {
      *pcl_wait_save += *feats_down_world;

      static int scan_wait_num = 0;
      scan_wait_num++;
      if (!pcl_wait_save->empty() && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
        pcd_index++;
        string all_points_dir(
          string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_dir << '\n';
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        pcl_wait_save->clear();
        scan_wait_num = 0;
      }
    }
  }
}

struct ImuForwardOdomState
{
  bool initialized = false;
  double state_time = 0.0;
  double last_pub_time = -1.0;
  V3D pos = Zero3d;
  M3D rot = Eye3d;
  V3D vel = Zero3d;
  V3D bg = Zero3d;
  V3D ba = Zero3d;
  V3D grav = Zero3d;
  V3D omg_body = Zero3d;
  V3D pos_corr = Zero3d;
  V3D rot_corr = Zero3d;
};

ImuForwardOdomState g_imu_fwd;

inline M3D projectSO3(const M3D & R)
{
  Eigen::Quaterniond q(R);
  q.normalize();
  return q.toRotationMatrix();
}

inline M3D so3ExpVec(const V3D & phi)
{
  const double theta = phi.norm();
  if (theta < 1e-10) {
    M3D skew;
    skew << SKEW_SYM_MATRX(phi);
    return M3D::Identity() + skew;
  }
  return Eigen::AngleAxisd(theta, phi / theta).toRotationMatrix();
}

inline V3D so3LogMat(const M3D & R)
{
  Eigen::AngleAxisd aa(R);
  double angle = aa.angle();
  if (!std::isfinite(angle)) {
    return Zero3d;
  }
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return aa.axis() * angle;
}

inline void stepImuForwardState(
  ImuForwardOdomState & st, const sensor_msgs::msg::Imu & imu_msg, bool decay_correction)
{
  const double t_imu = get_time_sec(imu_msg.header.stamp);
  const double dt = t_imu - st.state_time;
  if (!(dt > 0.0) || dt > 0.20) {
    if (dt > 0.20) {
      st.state_time = t_imu;
    }
    return;
  }
  V3D raw_omg(
    imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
  V3D raw_acc(
    imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
  if (check_satu) {
    for (int axis = 0; axis < 3; ++axis) {
      if (satu_gyro > 1e-6 && std::fabs(raw_omg(axis)) >= 0.99 * satu_gyro) {
        raw_omg(axis) = std::copysign(0.99 * satu_gyro, raw_omg(axis));
      }
      if (satu_acc > 1e-6 && std::fabs(raw_acc(axis)) >= 0.99 * satu_acc) {
        raw_acc(axis) = std::copysign(0.99 * satu_acc, raw_acc(axis));
      }
    }
  }
  const V3D omg = raw_omg - st.bg;
  const V3D acc_body = raw_acc * (G_m_s2 / std::max(1e-6, acc_norm)) - st.ba;
  const M3D dR_half = so3ExpVec(omg * (0.5 * dt));
  const M3D rot_mid = st.rot * dR_half;
  const V3D acc_world = rot_mid * acc_body + st.grav;
  st.pos += st.vel * dt + 0.5 * acc_world * (dt * dt);
  st.vel += acc_world * dt;
  st.rot = projectSO3(st.rot * so3ExpVec(omg * dt));
  st.omg_body = omg;
  st.state_time = t_imu;
  if (decay_correction) {
    if (imu_odom_blend_tau_s > 1e-4) {
      const double decay = std::exp(-dt / imu_odom_blend_tau_s);
      st.pos_corr *= decay;
      st.rot_corr *= decay;
    } else {
      st.pos_corr.setZero();
      st.rot_corr.setZero();
    }
  }
}

void publish_state_odometry(
  double stamp_sec, const V3D & pos, const M3D & rot, const V3D & vel_world, const V3D & omg_body,
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,
  const rclcpp::Publisher<ats_navigation_interfaces::msg::IdentifiedOdometry>::SharedPtr & identified_pub,
  const std::uint64_t instance_id)
{
  if (!(stamp_sec > g_imu_fwd.last_pub_time + 1e-7)) {
    return;
  }
  g_imu_fwd.last_pub_time = stamp_sec;

  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp = get_ros_time(stamp_sec);
  odomAftMapped.pose.pose.position.x = pos(0);
  odomAftMapped.pose.pose.position.y = pos(1);
  odomAftMapped.pose.pose.position.z = pos(2);
  Eigen::Quaterniond q(rot);
  q.normalize();
  odomAftMapped.pose.pose.orientation.x = q.coeffs()[0];
  odomAftMapped.pose.pose.orientation.y = q.coeffs()[1];
  odomAftMapped.pose.pose.orientation.z = q.coeffs()[2];
  odomAftMapped.pose.pose.orientation.w = q.coeffs()[3];

  const V3D vel_body = rot.transpose() * vel_world;
  odomAftMapped.twist.twist.linear.x = vel_body(0);
  odomAftMapped.twist.twist.linear.y = vel_body(1);
  odomAftMapped.twist.twist.linear.z = vel_body(2);
  odomAftMapped.twist.twist.angular.x = omg_body(0);
  odomAftMapped.twist.twist.angular.y = omg_body(1);
  odomAftMapped.twist.twist.angular.z = omg_body(2);

  ats_navigation_interfaces::msg::IdentifiedOdometry identified;
  identified.header = odomAftMapped.header;
  identified.instance_id = instance_id;
  identified.odometry = odomAftMapped;
  if (instance_id != 0) {
    identified_pub->publish(identified);
  }
  pubOdomAftMapped->publish(odomAftMapped);
}

void publish_odometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,
  const rclcpp::Publisher<ats_navigation_interfaces::msg::IdentifiedOdometry>::SharedPtr & identified_pub,
  const std::uint64_t instance_id)
{
  const double stamp_sec = publish_odometry_without_downsample ? time_current : lidar_end_time;
  publish_state_odometry(
    stamp_sec, kf_output.x_.pos, kf_output.x_.rot, kf_output.x_.vel, kf_output.x_.omg,
    pubOdomAftMapped, identified_pub, instance_id);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");

  // 回调只在主循环线程里执行（spin_some / spin_once），多线程执行器没有并行可用，只带来开销
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nh);

  readParameters(nh);
  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
    kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
  }

  kf_output.init_dyn_share_modified_3h(
    get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
  kf_output.set_maximum_iter(mapping_max_iter);
  Eigen::Matrix<double, 30, 30> P_init_output;  // = MD(24, 24)::Identity() * 0.01;
  reset_cov_output(P_init_output);
  kf_output.change_P(P_init_output);
  Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

  /*** ROS subscribe initialization ***/
  // 仅支持 Livox CustomMsg 输入（原 lidar_type=1）
  auto sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    lid_topic, rclcpp::SensorDataQoS(),
    [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livox_pcl_cbk(msg); });
  auto sub_imu =
    nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
  auto pub_laser_cloud_full_res =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 20);
  auto pub_laser_cloud_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
  auto pub_odom_aft_mapped =
    nh->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 20);
  auto pub_identified_odom =
    nh->create_publisher<ats_navigation_interfaces::msg::IdentifiedOdometry>(
    "aft_mapped_to_init_identified", 20);
  std::uint64_t instance_id = 0;
  const auto startup_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (std::chrono::steady_clock::is_steady && startup_ns > 0) {
    instance_id = static_cast<std::uint64_t>(startup_ns);
  } else {
    RCLCPP_ERROR(nh->get_logger(), "steady clock unavailable; identified odometry disabled");
  }
  g_imu_forward_odom_hook = [&](const sensor_msgs::msg::Imu & imu_msg) {
    if (!imu_odom_en || !g_imu_fwd.initialized) {
      return;
    }
    stepImuForwardState(g_imu_fwd, imu_msg, true);
    const double min_period = 1.0 / std::max(10.0, imu_odom_rate_hz);
    if (g_imu_fwd.state_time - g_imu_fwd.last_pub_time >= min_period - 1e-5) {
      const V3D disp_pos = g_imu_fwd.pos + g_imu_fwd.pos_corr;
      const M3D disp_rot = projectSO3(g_imu_fwd.rot * so3ExpVec(g_imu_fwd.rot_corr));
      publish_state_odometry(
        g_imu_fwd.state_time, disp_pos, disp_rot, g_imu_fwd.vel, g_imu_fwd.omg_body,
        pub_odom_aft_mapped, pub_identified_odom, instance_id);
    }
  };

  //------------------------------------------------------------------------------------------------------
  signal(SIGINT, SigHandle);
  // 组不出包时阻塞等下一条消息，不再 500 Hz 轮询。超时只为了检查 flg_exit：
  // SigHandle 替换了 rclcpp 的 SIGINT 处理，Ctrl-C 不会唤醒执行器
  const auto idle_wait = std::chrono::milliseconds(100);
  while (rclcpp::ok()) {
    if (flg_exit) break;
    executor.spin_some();
    if (sync_packages(Measures)) {
      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        {
          // imu_next = *(imu_deque.front());
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc *= -1;

          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_deque.pop();
            }
          }
        }
        G_m_s2 =
          std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      const auto scan_wall_t0 = std::chrono::steady_clock::now();
      /*** downsample the feature points in a scan ***/
      p_imu->Process(Measures, feats_undistort);
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      } else {
        // feats_undistort 与 Measures.lidar 共享同一块点云，这里必须拷贝后再排序，
        // 否则排序会改动 feats_undistort（原实现里 feats_undistort 是独立深拷贝）
        *feats_down_body = *Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      if (adaptive_downsample_en &&
        static_cast<int>(feats_down_body->points.size()) > max_surf_points && max_surf_points >= 2)
      {
        const std::size_t raw_n = feats_down_body->points.size();
        const std::size_t cap_n = static_cast<std::size_t>(max_surf_points);
        PointCloudXYZI thinned;
        thinned.points.reserve(cap_n);
        for (std::size_t i = 0; i + 1 < cap_n; ++i) {
          const std::size_t src_idx = (i * (raw_n - 1)) / (cap_n - 1);
          thinned.points.push_back(feats_down_body->points[src_idx]);
        }
        thinned.points.push_back(feats_down_body->points.back());
        thinned.width = static_cast<std::uint32_t>(thinned.points.size());
        thinned.height = 1;
        thinned.is_dense = feats_down_body->is_dense;
        feats_down_body->swap(thinned);
      }
      {
        time_seq = time_compressing<int>(feats_down_body);
        if (batch_update_size > 1 && time_seq.size() > 1U) {
          std::vector<int> batched;
          batched.reserve(time_seq.size());
          std::size_t pt_cursor = 0;
          std::size_t grp = 0;
          while (grp < time_seq.size()) {
            int acc_cnt = time_seq[grp];
            const double t_start =
              feats_down_body->points[pt_cursor].curvature / 1000.0;
            std::size_t end_pt = pt_cursor + static_cast<std::size_t>(acc_cnt);
            ++grp;
            while (grp < time_seq.size() && acc_cnt + time_seq[grp] <= batch_update_size) {
              const std::size_t cand_end_pt = end_pt + static_cast<std::size_t>(time_seq[grp]);
              const double t_end =
                feats_down_body->points[cand_end_pt - 1U].curvature / 1000.0;
              if (t_end - t_start > batch_max_dt_sec) {
                break;
              }
              acc_cnt += time_seq[grp];
              end_pt = cand_end_pt;
              ++grp;
            }
            batched.push_back(acc_cnt);
            pt_cursor = end_pt;
          }
          time_seq.swap(batched);
        }
        feats_down_size = feats_down_body->points.size();
      }

      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_output.x_.rot = rot_init;
          // kf_output.x_.rot; //.normalize();
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      /*** initialize the map ***/
      if (!init_map) {
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        for (const auto & point : *feats_down_world) {
          init_feats_world->points.emplace_back(point);
        }

        if (init_feats_world->size() >= init_map_size) {
          ivox_->AddPoints(init_feats_world->points);
          publish_init_map(pub_laser_cloud_map);
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        } else {
          init_map = false;
        }
        continue;
      }

      /*** ICP and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      /*** iterated state estimation ***/
      crossmat_list.resize(feats_down_size);
      pbody_list.resize(feats_down_size);
      // pbody_ext_list.reserve(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }
      {
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
              angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              is_first_frame = false;
              imu_upda_cov = true;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (!imu_deque.empty()) {
              bool last_imu = get_time_sec(imu_next.header.stamp) ==
                              get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                     !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                imu_upda_cov = true;
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                kf_output.predict(dt, Q_output, input_in, true, false);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);

                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                    kf_output.update_iterated_dyn_share_IMU();
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            double dt = time_current - time_predict_last_const;
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output, input_in, true, false);
            time_predict_last_const = time_current;

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            if (!kf_output.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, pub_identified_odom, instance_id);
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            idx += time_seq[k];
            // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                  if (dt_cov > 0.0) {
                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                    time_update_last = time_current;
                  }
                  kf_output.predict(dt, Q_output, input_in, true, false);
                }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                kf_output.update_iterated_dyn_share_IMU();
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      /******* Publish odometry downsample *******/
      {
        ImuForwardOdomState ekf_replayed;
        ekf_replayed.initialized = true;
        ekf_replayed.state_time =
          time_predict_last_const > 0.0 ? time_predict_last_const : lidar_end_time;
        ekf_replayed.pos = kf_output.x_.pos;
        ekf_replayed.rot = kf_output.x_.rot;
        ekf_replayed.vel = kf_output.x_.vel;
        ekf_replayed.bg = kf_output.x_.bg;
        ekf_replayed.ba = kf_output.x_.ba;
        ekf_replayed.grav = kf_output.x_.gravity;
        ekf_replayed.omg_body = kf_output.x_.omg;
        if (imu_odom_en) {
          for (const auto & imu_ptr : imu_deque) {
            if (imu_ptr && get_time_sec(imu_ptr->header.stamp) > ekf_replayed.state_time) {
              stepImuForwardState(ekf_replayed, *imu_ptr, false);
            }
          }
        }
        if (g_imu_fwd.initialized && imu_odom_en && imu_odom_blend_tau_s > 1e-4) {
          const V3D prev_disp_pos = g_imu_fwd.pos + g_imu_fwd.pos_corr;
          const M3D prev_disp_rot = projectSO3(g_imu_fwd.rot * so3ExpVec(g_imu_fwd.rot_corr));
          V3D dp = prev_disp_pos - ekf_replayed.pos;
          if (dp.norm() > 0.15) {
            dp.setZero();
          }
          V3D dr = so3LogMat(ekf_replayed.rot.transpose() * prev_disp_rot);
          if (dr.norm() > 0.15) {
            dr.setZero();
          }
          ekf_replayed.pos_corr = dp;
          ekf_replayed.rot_corr = dr;
        }
        ekf_replayed.last_pub_time = g_imu_fwd.last_pub_time;
        g_imu_fwd = ekf_replayed;

        if (!publish_odometry_without_downsample) {
          if (!imu_odom_en) {
            publish_odometry(pub_odom_aft_mapped, pub_identified_odom, instance_id);
          } else {
            const double pub_t = std::max(lidar_end_time, g_imu_fwd.state_time);
            const V3D disp_pos = g_imu_fwd.pos + g_imu_fwd.pos_corr;
            const M3D disp_rot = projectSO3(g_imu_fwd.rot * so3ExpVec(g_imu_fwd.rot_corr));
            publish_state_odometry(
              pub_t, disp_pos, disp_rot, g_imu_fwd.vel, g_imu_fwd.omg_body,
              pub_odom_aft_mapped, pub_identified_odom, instance_id);
          }
        }
      }

      /*** add the feature points to map ***/
      if (feats_down_size > 4) {
        MapIncremental();
      }
      const double scan_wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - scan_wall_t0).count();
      RCLCPP_INFO_THROTTLE(
        nh->get_logger(), *nh->get_clock(), 2000,
        "[point_lio 帧统计] wall_ms=%.3f surf_pts=%d batches=%zu effct_feat=%d imu_odom=%s",
        scan_wall_ms, feats_down_size, time_seq.size(), effct_feat_num,
        imu_odom_en ? "on" : "off");
      /******* Publish points *******/
      if (scan_pub_en || pcd_save_en) publish_frame_world(pub_laser_cloud_full_res);
    } else {
      executor.spin_once(idle_wait);
    }
  }
  //--------------------------save map-----------------------------------
  // 1. make sure you have enough memories
  // 2. noted that pcd save will influence the real-time performances
  if (!pcl_wait_save->empty() && pcd_save_en) {
    // 获取当前时间戳作为文件名的一部分
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3) << ms.count();
    
    string file_name = string("scans_") + ss.str() + string(".pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }
  return 0;
}
