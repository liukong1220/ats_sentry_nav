cmake_minimum_required(VERSION 3.10)
project(ats_navigation_interfaces)

# 未指定 CMAKE_BUILD_TYPE 时 colcon 不传 -O，整包以 -O0 编译（报告 5.1.6 H）；默认取 Release。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  "msg/LocalizationStatus.msg"
  "msg/ExecutionCommand.msg"
  "msg/GimbalYawStatus.msg"
  "msg/PlanningMapStatus.msg"
  "msg/PlanningMapSnapshot.msg"
  "msg/PlannerCandidate.msg"
  "msg/PlannerGoal.msg"
  "msg/PlannerStatus.msg"
  "msg/RelocalizationObservation.msg"
  "msg/SwerveTelemetry.msg"
  "msg/YawAuthorityRequest.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
