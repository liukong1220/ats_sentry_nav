#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// 仅保留 Livox CustomMsg 预处理：本项目只使用 Livox MID360（原 lidar_type=1），
// Velodyne/Ouster/Hesai 解析与 LOAM 特征提取已删除。
class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);

  PointCloudXYZI pl_full, pl_surf;
  int point_filter_num, N_SCANS;
  double blind, det_range;

  private:
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);
};
