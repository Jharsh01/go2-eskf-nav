cmake_minimum_required(VERSION 3.16)
project(go2_eskf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- Library: pure ESKF math, ROS-free, used by node, tools and tests
add_library(${PROJECT_NAME}_core src/eskf_core.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

# ---- ROS 2 node
add_executable(${PROJECT_NAME}_node src/eskf_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs)

# ---- Deterministic replay driver for cross-validation vs. the NumPy twin
add_executable(replay_eskf tools/replay_eskf.cpp)
target_link_libraries(replay_eskf ${PROJECT_NAME}_core)

# ---- Slip-model inference driver (Phase 3 cross-validation vs. the NumPy twin)
add_executable(slip_infer tools/slip_infer.cpp)
target_link_libraries(slip_infer ${PROJECT_NAME}_core)

# ---- Install
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS ${PROJECT_NAME}_node replay_eskf slip_infer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/eskf_reference.py
  scripts/cross_validate.py
  scripts/slip_reference.py
  scripts/cross_validate_slip.py
  scripts/train_slip_model.py
  scripts/metrics.py
  scripts/run_benchmark.py
  scripts/plot_trajectory.py
  scripts/square_test.py
  scripts/probe_leg_odom.py
  scripts/leg_odom_model.py
  scripts/make_terrain_world.py
  DESTINATION lib/${PROJECT_NAME})

# ---- Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_eskf_core test/test_eskf_core.cpp)
  target_link_libraries(test_eskf_core ${PROJECT_NAME}_core)
  ament_add_gtest(test_slip_model test/test_slip_model.cpp)
  target_link_libraries(test_slip_model ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3 rclcpp sensor_msgs nav_msgs geometry_msgs
                          tf2 tf2_ros tf2_geometry_msgs)
ament_package()
