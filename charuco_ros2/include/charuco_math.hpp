#ifndef CHARUCO_ROS2_CHARUCO_MATH_HPP
#define CHARUCO_ROS2_CHARUCO_MATH_HPP

#include <vector>

namespace cv_bridge
{
  class CvImage;
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
    CharucoMath();

    ~CharucoMath();

    void init(const CharucoRos2Context &cxt);

    void annotate_image(std::shared_ptr<cv_bridge::CvImage> color);

    void calculate_calibration(const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images);
  };
}
#endif //CHARUCO_ROS2_CHARUCO_MATH_HPP
