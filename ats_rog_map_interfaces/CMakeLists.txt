cmake_minimum_required(VERSION 3.10)
project(ats_rog_map_interfaces)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetRogMapProjection.srv"
  DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
