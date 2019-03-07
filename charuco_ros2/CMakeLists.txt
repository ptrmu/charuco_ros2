cmake_minimum_required(VERSION 3.5)
project(charuco_ros2)

#=============
# Setup
#=============

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon
if($ENV{CLION_IDE})
  message("Running inside CLion")
  set(charuco_ros2_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/charuco_ros2_msgs/share/charuco_ros2_msgs/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(charuco_ros2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Local includes
include_directories(
  include
  ${charuco_ros2_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${yaml_cpp_vendor_INCLUDE_DIRS}
)

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if(_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach(_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach()
endif()

#=============
# charuco_ros2 node
#=============

add_executable(${PROJECT_NAME}
  src/charuco_ros2_node.cpp
  src/charuco_ros2_context.cpp
  src/charuco_math.cpp)

ament_target_dependencies(${PROJECT_NAME}
  cv_bridge
  charuco_ros2_msgs
  OpenCV
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
  )

#=============
# Install
#=============

# Install targets
install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/charuco_ros2
)

# Install various directories
install(
  DIRECTORY
  launch
  DESTINATION share/charuco_ros2
)

#=============
# Run ament macros
#=============

ament_package()
