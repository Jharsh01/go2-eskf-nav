cmake_minimum_required(VERSION 3.16)
project(motion_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- Core library: ROS-free planner / grid / follower code
add_library(${PROJECT_NAME}_core
  src/occupancy_grid.cpp
  src/pure_pursuit.cpp
  src/planners/astar.cpp
  src/planners/rrt.cpp
  src/planners/prm.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---- Navigation node (ROS 2 wrapper)
add_executable(navigation_node src/navigation_node.cpp)
# Plain form (no PRIVATE/PUBLIC) required to match ament_target_dependencies.
target_link_libraries(navigation_node ${PROJECT_NAME}_core)
ament_target_dependencies(navigation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs
  tf2 tf2_geometry_msgs)

# ---- Benchmark executable (no Gazebo dependency)
add_executable(benchmark_node src/benchmark_node.cpp)
target_link_libraries(benchmark_node ${PROJECT_NAME}_core)
ament_target_dependencies(benchmark_node rclcpp)

# ---- Install
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include)

install(TARGETS navigation_node benchmark_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config worlds models maps rviz
  DESTINATION share/${PROJECT_NAME})

# ---- Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_planners      test/test_planners.cpp)
  target_link_libraries(test_planners ${PROJECT_NAME}_core)

  ament_add_gtest(test_pure_pursuit  test/test_pure_pursuit.cpp)
  target_link_libraries(test_pure_pursuit ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs sensor_msgs
                          visualization_msgs tf2 tf2_geometry_msgs)
ament_package()
