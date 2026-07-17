cmake_minimum_required(VERSION 3.10)
project(ats_navigation_interfaces)

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
  "msg/PlanningMapStatus.msg"
  "msg/PlannerGoal.msg"
  "msg/PlannerStatus.msg"
  "msg/RelocalizationObservation.msg"
  "msg/SwerveTelemetry.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
