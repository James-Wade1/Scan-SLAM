cmake_minimum_required(VERSION 3.8)
project(scan_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(scan_slam_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  scan_slam_msgs
)

add_library(${PROJECT_NAME}_lib SHARED
  # src/icp_scan_matcher.cpp
  src/correlative_scan_matcher.cpp
  src/slam_frontend.cpp
  src/correlative_matcher_lookup_table.cpp
  src/slam_backend.cpp
  # src/occupancy_grid.cpp
  # src/tf_manager.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

# Build the ROS node executable
add_executable(frontend_node src/frontend_main.cpp)
target_link_libraries(frontend_node ${PROJECT_NAME}_lib)
ament_target_dependencies(frontend_node ${DEPENDENCIES})

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install library and executable
install(TARGETS ${PROJECT_NAME}_lib frontend_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()