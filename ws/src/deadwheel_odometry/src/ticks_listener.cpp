#include <rclcpp/rclcpp.hpp>
#include "deadwheel_msgs/msg/deadwheel_ticks.hpp"
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
//#include <tf2_ros/transform_broadcaster.h>
//#include <geometry_msgs/msg/transform_stamped.hpp>

double normalizeAngleSigned(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

class TicksListener : public rclcpp::Node
{
  private : 
    rclcpp::Subscription<deadwheel_msgs::msg::DeadwheelTicks>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x__{0.0}, y__{0.0}, theta__{0.0};
    double vx{0}, vy{0}, omega{0};
    std::mutex mtx_;
  
    //Constantes, à ajouter les bonnes valeurs
    const double ENCODER_TICKS_PER_REVOLUTION [3] = {4096, 4096, 4096};
    const double DEADWHEEL_DIAMETER  = 0.0373;
    const double DEADWHEEL_CIRCUMFERENCE = (M_PI) * DEADWHEEL_DIAMETER;
    const double DEADWHEEL_DISTANCE = 0.1446; //distance entre les deux deadwheel principaux
    const double OFFSET = 0.063; //distance entre le side deadwheel et le centre de rotation du robot

    int64_t prevTicks[3] = {0, 0, 0};
    bool initialized_{false};
    rclcpp::Time prev_stamp_;

    void publishOdom(const rclcpp::Time& stamp)
    { 
      if (!initialized_)
        return;

      // Create quaternion from yaw
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, theta__);
      q.normalize();

      // Create Odometry message
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      // Pose
      odom.pose.pose.position.x = x__;
      odom.pose.pose.position.y = y__;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom.pose.covariance[0]  = 1e-3;  // x
      odom.pose.covariance[7]  = 1e-3;  // y
      odom.pose.covariance[35] = 5e-2;  // yaw

      // Twist (robot frame)
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = omega;
      odom.twist.covariance[0]  = 1e-2;  // vx
      odom.twist.covariance[7]  = 1e-2;  // vy
      odom.twist.covariance[35] = 1e-1;  // omega

      odom_pub_->publish(odom);

     /*  // TF transform
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";

      tf.transform.translation.x = x__;
      tf.transform.translation.y = y__;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tf);*/
    } 

  public:
    TicksListener() : Node("ticks_listener")
    {
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_deadwheels", 10);
      //tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      subscription_ = this->create_subscription<deadwheel_msgs::msg::DeadwheelTicks>(
      "deadwheel_ticks", 50,[this](deadwheel_msgs::msg::DeadwheelTicks::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);

        const rclcpp::Time stamp = this->now();

        if (!initialized_) {
          prevTicks[0] = msg->t0;
          prevTicks[1] = msg->t1;
          prevTicks[2] = msg->t2;
          prev_stamp_ = stamp;
          initialized_ = true;
         return;
        }

        double dt = (stamp - prev_stamp_).seconds();

        if (dt <= 1e-6 || !std::isfinite(dt)) {
          RCLCPP_WARN(this->get_logger(), "Bad dt (%.9f), skipping", dt);
          return;
        }

        prev_stamp_ = stamp;

        // calculs des delta ticks
        int64_t ticks0 = (msg->t0); 
        int64_t ticks1 = (msg->t1);
        int64_t ticks2 = (msg->t2);
        int64_t dRTicks = ticks0 - prevTicks[0];
        int64_t dLTicks = ticks1 - prevTicks[1];
        int64_t dSTicks = ticks2 - prevTicks[2];
        prevTicks[0] = ticks0;
        prevTicks[1] = ticks1;
        prevTicks[2] = ticks2;

        //calculs des déplacements selon les axes du robot
        double rightDist = static_cast<double>(dRTicks) * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[0];
        double leftDist = static_cast<double>(dLTicks) * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[1];
        double dxr = 0.5 * (rightDist + leftDist);
        double dangle = (rightDist - leftDist) / DEADWHEEL_DISTANCE;
        double dyr = (static_cast<double>(dSTicks) * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[2]) - OFFSET * dangle;
        double avgangle = theta__ + dangle/2; 

        //calculs des déplacements selon le world
        double c = std::cos(avgangle);
        double s = std::sin(avgangle);
        double dx = dxr*c - dyr*s;
        double dy = dyr*c + dxr*s;

        //mise à jour de la position du robot
        x__ += dx;
        y__ += dy;
        theta__ = normalizeAngleSigned(theta__ + dangle);

        //calculs de vitesse
        vx = dxr/dt;
        vy = dyr/dt;
        omega = dangle/dt;

        RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 500,  // milliseconds
        "x=%.3f y=%.3f", 
        x__, y__
        );

        publishOdom(stamp);
      }
    ); 
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TicksListener>());
  rclcpp::shutdown();
  return 0;
}
