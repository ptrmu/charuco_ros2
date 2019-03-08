#ifndef CHARUCO_ROS2_CONTEXT_HPP
#define CHARUCO_ROS2_CONTEXT_HPP
#include <string>

#include "context_macros.hpp"

namespace rclcpp
{
  class Node;
}

namespace charuco_ros2
{

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(               /* topic for republishing the image with axes added to fiducial markers  */\
  image_marked_pub_topic,  \
  "image_marked", std::string) \
  CXT_ELEM(               /* topic for republishing a captured image  */ \
  image_captured_pub_topic,  \
  "image_captured", std::string) \
  \
  CXT_ELEM(               /* topic for subscription to sensor_msgs::msg::Image */ \
  image_raw_sub_topic,  \
  "image_raw", std::string) \
  CXT_ELEM(               /* topic for subscription to image_captured topic */ \
  image_captured_sub_topic,  \
  "image_captured", std::string) \
  \
  CXT_ELEM(               /* topic for calibrate service */ \
  calibrate_srv_topic,  \
  "/fiducial_observations", std::string) \
  CXT_ELEM(               /* topic for capture service  */ \
  capture_srv_topic,  \
  "charuco_ros2_capture", std::string) \
  \
  CXT_ELEM(               /* non-zero => publish the image_marked at every frame  */ \
  publish_image_marked,  \
  1, int) \
  CXT_ELEM(               /* non-zero => publish or subscribe to image_captured  */ \
  enable_image_captured,  \
  0, int) \
  CXT_ELEM(               /* non-zero => publish not subscribe to image_captured message. useful for debugging with bags  */ \
  pub_not_sub_image_captured,  \
  1, int) \
                          /* charuco parameters */ \
  CXT_ELEM(               /* dictionary id  */ \
  aruco_dictionary_id,  \
  1, int) \
  CXT_ELEM(               /* non-zero => refine the detected markers  */ \
  refind_strategy,  \
  1, int) \
  CXT_ELEM(               /* number of squares in the x direction on the charuco board  */ \
  squares_x,  \
  5, int) \
  CXT_ELEM(               /* number of squares in the y direction on the charuco board  */ \
  squares_y,  \
  7, int) \
  CXT_ELEM(               /* length of a square on the charuco board  */ \
  square_length,  \
  0.16, double) \
  CXT_ELEM(               /* length of a marker on the charuco board  */ \
  marker_length,  \
  0.08, double) \
  \
  CXT_ELEM(               /* non-zero => publish the tf of the camera at every frame  */ \
  publish_tfs,  \
  1, int) \
  CXT_ELEM(               /* non-zero => publish the odometry of the camera at every frame  */ \
  publish_odom,  \
  1, int) \
  CXT_ELEM(               /* non-zero => debug mode, helpful for dealing with rviz when playing bags.  */ \
  stamp_msgs_with_current_time,  \
  1, int) \
  /* End of list */

#define CXT_MACRO_ALL_MEMBERS \
  /* End of list */

  struct CharucoRos2Context
  {
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_FIELD_DEF(n, a)
    CXT_MACRO_ALL_PARAMS

#undef CXT_MEMBER
#define CXT_MEMBER(n, a...) CXT_MEMBER_FIELD_DEF(n, a)
    CXT_MACRO_ALL_MEMBERS

    void load_parameters(rclcpp::Node &node);

    void validate_parameters();
  };
}

#endif //CHARUCO_ROS2_CONTEXT_HPP
