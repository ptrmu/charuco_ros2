cmake_minimum_required(VERSION 3.5)
project(charuco_ros2_msgs)

# Emulate colcon in CLion
if ($ENV{CLION_IDE})
  message("Running inside CLion")
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/bouncy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/bouncy/lib/libfastrtps.so")
endif ()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${std_msgs_INCLUDE_DIRS}
)

set(SRV_FILES
  "srv/CalibrateAction.srv"
  "srv/CaptureAction.srv"
  )

# Generate ROS interfaces
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
)

ament_package()
