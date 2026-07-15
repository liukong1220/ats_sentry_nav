cmake_minimum_required(VERSION 3.10)
project(ats_rog_map_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ats_rog_map_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_optimizer REQUIRED)

add_library(${PROJECT_NAME}_core src/ground_projection_fusion.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  ats_rog_map_interfaces geometry_msgs nav_msgs tf2 tf2_geometry_msgs trajectory_optimizer
)

add_executable(ats_rog_map_adapter_node src/ats_rog_map_adapter_node.cpp)
target_link_libraries(ats_rog_map_adapter_node ${PROJECT_NAME}_core)
ament_target_dependencies(ats_rog_map_adapter_node
  ats_rog_map_interfaces rclcpp std_msgs tf2_ros trajectory_optimizer
)

install(TARGETS ${PROJECT_NAME}_core ats_rog_map_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_ground_projection_fusion test/test_ground_projection_fusion.cpp)
  target_link_libraries(test_ground_projection_fusion ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(ats_rog_map_interfaces nav_msgs trajectory_optimizer)
ament_package()
