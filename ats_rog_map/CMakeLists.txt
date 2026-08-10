cmake_minimum_required(VERSION 3.10)
project(ats_rog_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 未指定 CMAKE_BUILD_TYPE 时 colcon 不会传任何 -O，整包以 -O0 编译。本包的
# getGridType/getESDFDistance 是 Eigen 密集的逐格调用（实测 60000 次/投影），
# -O0 下无内联、无向量化，投影 p50 达到 1.5 s，远超 cloud_timeout_sec=2.0 s 的
# 余量。这里只补默认优化等级，不改任何超时、deadline、lease、控制周期或门禁。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ats_rog_map_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ROG_MAP_CORE_SOURCES
  include/rog_map/rog_map_core/raycaster.cpp
  src/rog_map/counter_map.cpp
  src/rog_map/esdf_map.cpp
  src/rog_map/inf_map.cpp
  src/rog_map/prob_map.cpp
  src/rog_map/rog_map.cpp
  src/rog_map/sliding_map.cpp
)

add_library(${PROJECT_NAME}_core ${ROG_MAP_CORE_SOURCES})
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME}_core PUBLIC
  ORIGIN_AT_CORNER
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)

add_executable(ats_rog_map_node src/ats_rog_map_node.cpp)
target_link_libraries(ats_rog_map_node ${PROJECT_NAME}_core)
ament_target_dependencies(ats_rog_map_node ats_rog_map_interfaces visualization_msgs)

install(TARGETS ${PROJECT_NAME}_core ats_rog_map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE NOTICE README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  include(CTest)

  add_executable(prob_map_log_odds_fusion_test test/prob_map_log_odds_fusion_test.cpp)
  target_link_libraries(prob_map_log_odds_fusion_test ${PROJECT_NAME}_core)
  add_test(NAME prob_map_log_odds_fusion_test COMMAND prob_map_log_odds_fusion_test)

  add_executable(prob_map_stale_decay_test test/prob_map_stale_decay_test.cpp)
  target_link_libraries(prob_map_stale_decay_test ${PROJECT_NAME}_core)
  add_test(NAME prob_map_stale_decay_test COMMAND prob_map_stale_decay_test)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_split_robot_sensor_pose test/test_split_robot_sensor_pose.cpp)
  target_link_libraries(test_split_robot_sensor_pose ${PROJECT_NAME}_core)
  target_compile_definitions(test_split_robot_sensor_pose PRIVATE
    ATS_ROG_MAP_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/test/rog_map_test.yaml")

  ament_add_gtest(test_fault_authorization test/test_fault_authorization_test.cpp)
  target_include_directories(test_fault_authorization PRIVATE include)

  ament_add_gtest(test_rog_map_engine_snapshot test/test_rog_map_engine_snapshot.cpp)
  target_link_libraries(test_rog_map_engine_snapshot ${PROJECT_NAME}_core)
  target_compile_definitions(test_rog_map_engine_snapshot PRIVATE
    ATS_ROG_MAP_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/test/rog_map_test.yaml")
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3)
ament_package()
