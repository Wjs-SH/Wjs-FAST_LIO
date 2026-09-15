cmake_minimum_required(VERSION 3.10)
project(livox_ros_driver2)

# Messages-only build: FAST-LIO depends on livox_ros_driver2 solely for the
# CustomMsg/CustomPoint message types (Livox Avia path). We use Hesai/Velodyne, so
# the actual Livox driver node and its hardware SDK (liblivox_lidar_sdk_shared.so)
# are NOT needed. This minimal CMake generates only the two interfaces under the
# livox_ros_driver2 package name that FAST-LIO includes.
# Full upstream driver: https://github.com/Livox-SDK/livox_ros_driver2

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
