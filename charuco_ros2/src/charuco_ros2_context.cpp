
#include "charuco_ros2_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace charuco_ros2
{
  CharucoRos2Context::CharucoRos2Context(rclcpp::Node &node) :
    node_{node}
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(node_, (*this), n, t, d)
    CXT_MACRO_INIT_PARAMETERS(CHARUCO_ROS_ALL_PARAMS, validate_parameters)


#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED(node, CHARUCO_ROS_ALL_PARAMS, validate_parameters)
  }

  void CharucoRos2Context::validate_parameters()
  {
    RCLCPP_INFO(node_.get_logger(), "CharucoRos2 Parameters");

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, node_.get_logger(), (*this), n, t, d)
    CHARUCO_ROS_ALL_PARAMS
  }
}

