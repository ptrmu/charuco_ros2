#ifndef CHARUCO_ROS2_CONTEXT_HPP
#define CHARUCO_ROS2_CONTEXT_HPP

#include <string>

#include "ros2_shared/context_macros.hpp"

namespace rclcpp
{
  class Node;
}

namespace charuco_ros2
{
#define CHARUCO_ROS_ALL_PARAMS \
  CXT_MACRO_MEMBER(image_marked_pub_topic, std::string, "image_marked")         /* topic for republishing the image with axes added to fiducial markers  */\
  CXT_MACRO_MEMBER(image_captured_pub_topic, std::string, "image_captured")     /* topic for republishing a captured image  */ \
  \
  CXT_MACRO_MEMBER( image_raw_sub_topic, std::string, "image_raw")              /* topic for subscription to sensor_msgs::msg::Image */ \
  CXT_MACRO_MEMBER(image_captured_sub_topic, std::string, "image_captured")     /* topic for subscription to image_captured topic */ \
  \
  CXT_MACRO_MEMBER( calibrate_srv_topic, std::string, "charuco_ros2_calibrate") /* topic for calibrate service */ \
  CXT_MACRO_MEMBER(capture_srv_topic, std::string, "charuco_ros2_capture")      /* topic for capture service  */ \
  \
  CXT_MACRO_MEMBER(publish_image_marked, int, 1)          /* non-zero => publish the image_marked at every frame  */ \
  CXT_MACRO_MEMBER(enable_image_captured, int,  0)        /* non-zero => publish or subscribe to image_captured  */ \
  CXT_MACRO_MEMBER(pub_not_sub_image_captured, int, 1)    /* non-zero => publish not subscribe to image_captured message. useful for debugging with bags  */ \
                          /* charuco parameters */ \
  CXT_MACRO_MEMBER(aruco_dictionary_id, int, 5)           /* dictionary id  */ \
  CXT_MACRO_MEMBER(refind_strategy, int, 1)               /* non-zero => refine the detected markers  */ \
  CXT_MACRO_MEMBER( squares_x, int, 12)                   /* number of squares in the x direction on the charuco board  */ \
  CXT_MACRO_MEMBER(squares_y, int, 9)                     /* number of squares in the y direction on the charuco board  */ \
  CXT_MACRO_MEMBER( square_length, float, 0.030)          /* length of a square on the charuco board  */ \
  CXT_MACRO_MEMBER(marker_length, float, 0.0225)          /* length of a marker on the charuco board  */ \
  \
  CXT_MACRO_MEMBER(publish_tfs, int, 1)                   /* non-zero => publish the tf of the camera at every frame  */ \
  CXT_MACRO_MEMBER(publish_odom, int, 1)                  /* non-zero => publish the odometry of the camera at every frame  */ \
  CXT_MACRO_MEMBER(stamp_msgs_with_current_time, int,  1) /* non-zero => debug mode, helpful for dealing with rviz when playing bags.  */ \
  /* End of list */

  struct CharucoRos2Context
  {
    rclcpp::Node &node_;

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    CHARUCO_ROS_ALL_PARAMS

    explicit CharucoRos2Context(rclcpp::Node &node);

    void validate_parameters();
  };
}

#endif //CHARUCO_ROS2_CONTEXT_HPP
