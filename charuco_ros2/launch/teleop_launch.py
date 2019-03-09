import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Full teleop launch: driver, base, joystick, etc.


def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(cmd=['rviz2', '-d', 'src/fiducial_vlam/fiducial_vlam/cfg/default.rviz'], output='screen'),
        ExecuteProcess(cmd=['rviz2'], output='screen'),
        Node(package='tello_driver', node_executable='tello_driver', output='screen'),
        Node(package='tello_driver', node_executable='tello_joy', output='screen'),
        Node(package='joy', node_executable='joy_node', output='screen'),
        # Node(package='charuco_ros2', node_executable='charuco_ros2_node', output='screen'),
    ])
