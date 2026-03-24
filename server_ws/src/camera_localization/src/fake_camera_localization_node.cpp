#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class CameraLocalizationNode : public rclcpp::Node
{
public:
  CameraLocalizationNode()
  : Node("camera_localization_node")
  {
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = this->declare_parameter<std::string>("child_frame_id", "base_link");

    // Fake pose parameters for testing
    fake_x_ = this->declare_parameter<double>("fake_x", 1.0);
    fake_y_ = this->declare_parameter<double>("fake_y", 0.5);
    fake_yaw_ = this->declare_parameter<double>("fake_yaw", 0.2);

    // Covariances
    cov_x_ = this->declare_parameter<double>("cov_x", 0.02);
    cov_y_ = this->declare_parameter<double>("cov_y", 0.02);
    cov_yaw_ = this->declare_parameter<double>("cov_yaw", 0.05);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/camera/global_pose", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CameraLocalizationNode::publishPose, this));

    RCLCPP_INFO(this->get_logger(),
      "camera_localization_node started. Publishing fake global pose on /camera/global_pose");
  }

private:
  void publishPose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    msg.pose.pose.position.x = fake_x_;
    msg.pose.pose.position.y = fake_y_;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, fake_yaw_);
    q.normalize();

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    // 6x6 covariance matrix in row-major order:
    // [x, y, z, roll, pitch, yaw]
    for (auto & v : msg.pose.covariance) {
      v = 0.0;
    }

    msg.pose.covariance[0] = cov_x_;      // x
    msg.pose.covariance[7] = cov_y_;      // y
    msg.pose.covariance[14] = 1e6;        // z unused
    msg.pose.covariance[21] = 1e6;        // roll unused
    msg.pose.covariance[28] = 1e6;        // pitch unused
    msg.pose.covariance[35] = cov_yaw_;   // yaw

    pose_pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double publish_rate_hz_;
  std::string frame_id_;
  std::string child_frame_id_;

  double fake_x_;
  double fake_y_;
  double fake_yaw_;

  double cov_x_;
  double cov_y_;
  double cov_yaw_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}