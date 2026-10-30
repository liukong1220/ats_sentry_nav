cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

# 未指定 CMAKE_BUILD_TYPE 时 colcon 不传 -O，整包以 -O0 编译（报告 5.1.6 H）；默认取 Release。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)

rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
