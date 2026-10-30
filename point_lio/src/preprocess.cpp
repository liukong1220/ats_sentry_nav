#include "preprocess.h"

Preprocess::Preprocess() : point_filter_num(1), N_SCANS(6), blind(0.01), det_range(1000) {}

Preprocess::~Preprocess() {}

void Preprocess::process(
  const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg, PointCloudXYZI::Ptr & pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)
{
  pl_surf.clear();
  pl_full.clear();
  int plsize = msg->point_num;

  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  uint valid_num = 0;

  for (uint i = 1; i < plsize; i++) {
    if (
      (msg->points[i].line < N_SCANS) &&
      ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
      valid_num++;
      if (valid_num % point_filter_num == 0) {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;  // z; //
        pl_full[i].curvature =
          msg->points[i].offset_time /
          float(1000000);  // use curvature as time of each laser points, curvature unit: ms
        double dist =
          pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z;
        if (dist < blind * blind || dist > det_range * det_range) continue;
        if (((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
             (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
             (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))) {
          pl_surf.push_back(pl_full[i]);
        }
      }
    }
  }
}
