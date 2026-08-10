cmake_minimum_required(VERSION 3.18)
project(ats_swerve_mpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 与 ats_rog_map 同因：未指定 CMAKE_BUILD_TYPE 时整包以 -O0 编译。iLQR 的
# rollout/jacobian/backward pass 全是小矩阵 Eigen 运算，-O0 下没有内联与
# 向量化。这里只补默认优化等级，不改 20 Hz 控制周期、MPC 约束或任何门禁。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_CODEGEN OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PROFILING ON CACHE BOOL "" FORCE)
set(OSQP_ENABLE_INTERRUPT ON CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/osqp
                 ${CMAKE_CURRENT_BINARY_DIR}/osqp EXCLUDE_FROM_ALL)
if(TARGET osqpstatic AND NOT TARGET osqp::osqpstatic)
  add_library(osqp::osqpstatic ALIAS osqpstatic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ats_navigation_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/ats_swerve_mpc_node.cpp
  src/control_cycle_snapshot.cpp
  src/control_cycle_telemetry.cpp
  src/se2_model.cpp
  src/se2_mpc_controller.cpp
  src/ltv_qp_problem.cpp
  src/ltv_qp_solver.cpp
  src/ltv_qp_osqp_solver.cpp
  src/trajectory_tracker.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen osqp::osqpstatic)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
ament_target_dependencies(${PROJECT_NAME}_lib
  ats_navigation_interfaces geometry_msgs nav_msgs rclcpp std_srvs std_msgs tf2 tf2_geometry_msgs
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
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/osqp/LICENSE
              ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/osqp/NOTICE
        DESTINATION share/${PROJECT_NAME}/third_party/osqp)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/osqp/include/public/
        DESTINATION include/osqp FILES_MATCHING PATTERN "*.h")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_se2_mpc_controller test/test_se2_mpc_controller.cpp)
  target_link_libraries(test_se2_mpc_controller ${PROJECT_NAME}_lib)

  ament_add_gtest(test_se2_model test/test_se2_model.cpp)
  target_link_libraries(test_se2_model ${PROJECT_NAME}_lib)

  ament_add_gtest(test_zero_speed_guard test/test_zero_speed_guard.cpp)
  target_link_libraries(test_zero_speed_guard ${PROJECT_NAME}_lib)

  ament_add_gtest(test_ltv_qp_problem test/test_ltv_qp_problem.cpp)
  target_link_libraries(test_ltv_qp_problem ${PROJECT_NAME}_lib)

  ament_add_gtest(test_ltv_qp_solver test/test_ltv_qp_solver.cpp)
  target_link_libraries(test_ltv_qp_solver ${PROJECT_NAME}_lib)

  ament_add_gtest(test_ltv_qp_osqp_solver test/test_ltv_qp_osqp_solver.cpp)
  target_link_libraries(test_ltv_qp_osqp_solver ${PROJECT_NAME}_lib)

  ament_add_gtest(test_ltv_qp_convergence_fixture test/test_ltv_qp_convergence_fixture.cpp)
  target_link_libraries(test_ltv_qp_convergence_fixture ${PROJECT_NAME}_lib)

  ament_add_gtest(test_control_cycle_snapshot test/test_control_cycle_snapshot.cpp)
  target_link_libraries(test_control_cycle_snapshot ${PROJECT_NAME}_lib)

  ament_add_gtest(test_control_cycle_telemetry test/test_control_cycle_telemetry.cpp)
  target_link_libraries(test_control_cycle_telemetry ${PROJECT_NAME}_lib)

  ament_add_gtest(test_trajectory_tracker test/test_trajectory_tracker.cpp)
  target_link_libraries(test_trajectory_tracker ${PROJECT_NAME}_lib)

  ament_add_gtest(test_emergency_stop_watchdog test/test_emergency_stop_watchdog.cpp)
  target_link_libraries(test_emergency_stop_watchdog ${PROJECT_NAME}_lib)

  ament_add_gtest(test_mpc_localization_gate test/test_mpc_localization_gate.cpp)
  target_link_libraries(test_mpc_localization_gate ${PROJECT_NAME}_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(Eigen3 eigen3_cmake_module)
ament_package()
