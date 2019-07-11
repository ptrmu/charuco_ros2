
#include "rclcpp/rclcpp.hpp"

#include "charuco_math.hpp"
#include "charuco_ros2_context.hpp"

#include "charuco_ros2_msgs/srv/calibrate.hpp"
#include "charuco_ros2_msgs/srv/capture.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

namespace charuco_ros2
{
// ==============================================================================
// CharucoRos2Node class
// ==============================================================================

  class CharucoRos2Node : public rclcpp::Node
  {
    rclcpp::Logger ros_logger_inst;
    rclcpp::Logger &ros_logger;

    CharucoRos2Context cxt_;
    CharucoMath cm_;

    sensor_msgs::msg::Image::UniquePtr last_image_msg_{};
    std::vector<cv_bridge::CvImagePtr> captured_images_{};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_captured_pub_{};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_{};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_captured_sub_{};

    rclcpp::Service<charuco_ros2_msgs::srv::Calibrate>::SharedPtr calibrate_srv_{};
    rclcpp::Service<charuco_ros2_msgs::srv::Capture>::SharedPtr capture_srv_{};

    constexpr static double CAPTURED_IMAGE_EXPIRATION_SEC = 0.5;

  public:
    CharucoRos2Node()
      : Node("charuco_ros2_node"),
        ros_logger_inst{get_logger()}, ros_logger{ros_logger_inst},
        cxt_{*this}, cm_{ros_logger, cxt_}
    {
      // ROS publishers. Initialize after parameters have been loaded.
      if (cxt_.publish_image_marked_) {
        image_marked_pub_ = create_publisher<sensor_msgs::msg::Image>(
          cxt_.image_marked_pub_topic_, 16);
      }

      if (cxt_.enable_image_captured_ && cxt_.pub_not_sub_image_captured_) {
        image_captured_pub_ = create_publisher<sensor_msgs::msg::Image>(
          cxt_.image_captured_pub_topic_, 16);
      }


      // ROS subscriptions
      image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        cxt_.image_raw_sub_topic_,
        16,
        [this](sensor_msgs::msg::Image::UniquePtr msg) -> void
        {
          // Make a copy of this image, annotate it, and publish it.
          if (cxt_.publish_image_marked_) {
            auto color{cv_bridge::toCvCopy(*msg)};

            cm_.annotate_image(color);

            auto marked_image_msg{color->toImageMsg()};
            marked_image_msg->header = msg->header;
            image_marked_pub_->publish(*marked_image_msg);
          }

          // Save the last image. Take ownership of the image message. ROS has
          // passed this UniquePtr to us by value so our copy owns the message.
          // Normally the message would get destructed when this method returns
          // but instead we swap it with nullptr or the last message so it will
          // get destructed later.
          this->last_image_msg_.swap(msg);
        });

      image_captured_sub_ = create_subscription<sensor_msgs::msg::Image>(
        cxt_.image_captured_sub_topic_,
        16,
        [this](const sensor_msgs::msg::Image::UniquePtr msg) -> void
        {
        }
      );


      // ROS services
      calibrate_srv_ = create_service<charuco_ros2_msgs::srv::Calibrate>(
        cxt_.calibrate_srv_topic_,
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<charuco_ros2_msgs::srv::Calibrate::Request> request,
               std::shared_ptr<charuco_ros2_msgs::srv::Calibrate::Response> response) -> void
        {
          response->rc = cm_.calculate_calibration(captured_images_);
        });

      capture_srv_ = create_service<charuco_ros2_msgs::srv::Capture>(
        cxt_.capture_srv_topic_,
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<charuco_ros2_msgs::srv::Capture::Request> request,
               std::shared_ptr<charuco_ros2_msgs::srv::Capture::Response> response) -> void
        {
          // If the image isn't fresh enough or if there is no image, then return an error.
          if (!last_image_msg_) {
            response->rc = response->NO_IMAGE_AVAILABLE;
            return;
          }
//          auto age = now().seconds() - rclcpp::Time(last_image_msg_->header.stamp).seconds();
//          if (age > CAPTURED_IMAGE_EXPIRATION_SEC) {
//            response->rc = response->IMAGE_IS_OLD;
//            return;
//          }

          // Add a copy of this captured image to the list already captured.
          captured_images_.emplace_back(cv_bridge::toCvCopy(*last_image_msg_));

          // Publish this captured image if requested
          if (cxt_.enable_image_captured_ && cxt_.pub_not_sub_image_captured_) {
            image_captured_pub_->publish(*last_image_msg_);
          }

          // As an extra precaution, free the image so it can't be captured again.
          last_image_msg_.reset();

          // return success
          response->rc = response->OK;

          RCLCPP_INFO(this->get_logger(), "Frame captured: %d", captured_images_.size());
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
