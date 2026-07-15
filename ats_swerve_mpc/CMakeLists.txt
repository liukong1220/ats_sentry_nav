cmake_minimum_required(VERSION 3.10)
project(ats_swerve_mpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/ats_swerve_mpc_node.cpp
  src/se2_mpc_controller.cpp
  src/trajectory_tracker.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_lib
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_se2_mpc_controller test/test_se2_mpc_controller.cpp)
  target_link_libraries(test_se2_mpc_controller ${PROJECT_NAME}_lib)

  ament_add_gtest(test_trajectory_tracker test/test_trajectory_tracker.cpp)
  target_link_libraries(test_trajectory_tracker ${PROJECT_NAME}_lib)

  ament_add_gtest(test_emergency_stop_watchdog test/test_emergency_stop_watchdog.cpp)
  target_link_libraries(test_emergency_stop_watchdog ${PROJECT_NAME}_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(Eigen3 eigen3_cmake_module)
ament_package()
