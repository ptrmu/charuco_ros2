#ifndef CHARUCO_ROS2_CHARUCO_MATH_HPP
#define CHARUCO_ROS2_CHARUCO_MATH_HPP

#include <vector>

#include "charuco_ros2_msgs/srv/calibrate.hpp"

namespace cv_bridge
{
  class CvImage;
}
namespace rclcpp
{
  class Logger;
}

namespace charuco_ros2
{
  class CharucoRos2Context;

// ==============================================================================
// CharucoMath class
// ==============================================================================

  class CharucoMath
  {
    class CvCharucoMath;

    std::unique_ptr<CvCharucoMath> cv_;

  public:
    explicit CharucoMath(rclcpp::Logger &logger, const CharucoRos2Context &cxt);

    ~CharucoMath();

    void annotate_image(std::shared_ptr<cv_bridge::CvImage> &color);

    charuco_ros2_msgs::srv::Calibrate::Response::_rc_type calculate_calibration(
      const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images);
  };
}
#endif //CHARUCO_ROS2_CHARUCO_MATH_HPP
