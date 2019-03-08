
#include "rclcpp/rclcpp.hpp"

#include "charuco_ros2_msgs/srv/calibrate.hpp"
#include "charuco_ros2_msgs/srv/capture.hpp"

namespace charuco_ros2
{
  class CharucoRos2Node : public rclcpp::Node
  {

    rclcpp::Service<charuco_ros2_msgs::srv::Calibrate>::SharedPtr calibrate_srv_;
    rclcpp::Service<charuco_ros2_msgs::srv::Capture>::SharedPtr capture_srv_;

  public:
    CharucoRos2Node()
      : Node("charuco_ros2_node")
    {

      // ROS services
      calibrate_srv_ = create_service<charuco_ros2_msgs::srv::Calibrate>(
        "charuco_ros2_calibrate",
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<charuco_ros2_msgs::srv::Calibrate::Request> request,
               std::shared_ptr<charuco_ros2_msgs::srv::Calibrate::Response> response) -> void
        {
        });

      capture_srv_ = create_service<charuco_ros2_msgs::srv::Capture>(
        "charuco_ros2_capture",
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<charuco_ros2_msgs::srv::Capture::Request> request,
               std::shared_ptr<charuco_ros2_msgs::srv::Capture::Response> response) -> void
        {
          response->rc = response->NO_IMAGE_AVAILABLE;
        });

      RCLCPP_INFO(get_logger(), "charuco_ros2 node ready");
    }
  };
}

// ==============================================================================
// main()
// ==============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<charuco_ros2::CharucoRos2Node>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
