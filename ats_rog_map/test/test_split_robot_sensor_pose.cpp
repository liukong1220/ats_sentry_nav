// Copyright 2026

#include <gtest/gtest.h>

#include "rog_map/rog_map.h"

namespace
{

class TestMap final : public rog_map::ROGMap
{
public:
  TestMap()
  {
    cfg_ = rog_map::Config(ATS_ROG_MAP_TEST_CONFIG);
    init();
  }

  void update(
    const rog_map::PointCloud & cloud, const rog_map::Pose & robot_pose,
    const rog_map::Pose & sensor_pose)
  {
    updateRobotState(robot_pose);
    updateProbMap(cloud, sensor_pose, robot_pose.first);
  }

private:
  const double getSystemWalltimeNow() override
  {
    return 0.0;
  }
};

TEST(AtsRogMap, KeepsRobotCenterSeparateFromSensorRayOrigin)
{
  TestMap map;
  rog_map::PointCloud cloud;
  pcl::PointXYZI point;
  point.x = 1.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 1.0F;
  cloud.push_back(point);

  const rog_map::Pose robot_pose{
    rog_map::Vec3f(0.0F, 0.0F, 0.0F), super_utils::Quatf::Identity()};
  const rog_map::Pose sensor_pose{
    rog_map::Vec3f(0.2F, 0.0F, 0.0F), super_utils::Quatf::Identity()};
  map.update(cloud, robot_pose, sensor_pose);
  map.update(cloud, robot_pose, sensor_pose);

  EXPECT_NEAR(map.getRobotState().p.x(), 0.0, 1e-6);
  EXPECT_NEAR(map.getRobotState().p.y(), 0.0, 1e-6);
  EXPECT_TRUE(map.isOccupied(rog_map::Vec3f(1.0F, 0.0F, 0.0F)));
}

}  // namespace
