cmake_minimum_required(VERSION 3.8)
project(pixhawk_sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)

# C++ Executable: vio_px4_bridge
add_executable(vio_px4_bridge src/vio_px4_bridge.cpp)
ament_target_dependencies(vio_px4_bridge
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  px4_msgs
)

# Install C++ binaries
install(TARGETS
  vio_px4_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/pixhawk_imu_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and configs
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
