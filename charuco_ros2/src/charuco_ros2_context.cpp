
#include "charuco_ros2_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace charuco_ros2
{
  void CharucoRos2Context::load_parameters(rclcpp::Node &node)
  {
    // Read parameters from the command line. NOTE: the get_parameter() method will
    // not modify the member element if the parameter does not exist on the command line.
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_LOAD_PARAM(n, a)
    CXT_MACRO_ALL_PARAMS

    validate_parameters();
  }

  void CharucoRos2Context::validate_parameters()
  {
  }
}

