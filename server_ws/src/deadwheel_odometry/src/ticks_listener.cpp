#include <rclcpp/rclcpp.hpp>
#include "custom_msgs/msg/deadwheel_ticks.hpp"
#include <cmath>
#include <cstdlib>
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
    rclcpp::Subscription<custom_msgs::msg::DeadwheelTicks>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x__{0.0}, y__{0.0}, theta__{0.0};
    double vx{0}, vy{0}, omega{0};
    std::mutex mtx_;

    //Constantes, à ajouter les bonnes valeurs
    const double ENCODER_TICKS_PER_REVOLUTION [3] = {4096, 4096, 4096};
    const double DEADWHEEL_DIAMETER  = 0.0384; //0.0373, 0.0366
    const double SDEADWHEEL_DIAMETER  = 0.0384;
    const double DEADWHEEL_CIRCUMFERENCE = (M_PI) * DEADWHEEL_DIAMETER;
    const double SDEADWHEEL_CIRCUMFERENCE = (M_PI) * SDEADWHEEL_DIAMETER;
    const double DEADWHEEL_DISTANCE = 0.1496; //distance entre les deux deadwheel principaux, anciennement 0.1446
    const double OFFSET = -0.063; //distance entre le side deadwheel et le centre de rotation du robot
    const double RIGHT_DEADWHEEL_SIGN = -1.0;
    const double LEFT_DEADWHEEL_SIGN = -1.0;
    const double SIDE_DEADWHEEL_SIGN = -1.0;

    int64_t prevTicks[3] = {0, 0, 0};
    bool initialized_{false};
    rclcpp::Time prev_stamp_;
    rclcpp::Time prev_receive_stamp_;
    bool stamp_diagnostics_{false};
    double max_stamp_offset_sec_{1.0};
    int64_t max_tick_delta_{50000};

    bool getMessageStamp(
      const custom_msgs::msg::DeadwheelTicks::SharedPtr& msg,
      const rclcpp::Time& receive_stamp,
      rclcpp::Time& source_stamp,
      bool& source_stamp_is_ros_synced)
    {
      if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
        source_stamp = receive_stamp;
        source_stamp_is_ros_synced = false;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "deadwheel_ticks header stamp is zero; using receive time");
        return true;
      }

      source_stamp = rclcpp::Time(msg->header.stamp);
      const double offset = (receive_stamp - source_stamp).seconds();
      source_stamp_is_ros_synced = std::isfinite(offset) && std::abs(offset) <= max_stamp_offset_sec_;
      if (!source_stamp_is_ros_synced) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "deadwheel_ticks stamp is %.3f s from receive time; using receive time",
          offset);
      }

      return true;
    }

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
      const double initial_x = this->declare_parameter<double>("initial_x", 0.0);
      const double initial_y = this->declare_parameter<double>("initial_y", 0.0);
      const double initial_yaw_deg = this->declare_parameter<double>("initial_yaw_deg", 0.0);
      stamp_diagnostics_ = this->declare_parameter<bool>("stamp_diagnostics", false);
      max_stamp_offset_sec_ = this->declare_parameter<double>("max_stamp_offset_sec", 1.0);
      max_tick_delta_ = this->declare_parameter<int64_t>("max_tick_delta", 50000);

      x__ = initial_x;
      y__ = initial_y;
      theta__ = normalizeAngleSigned(initial_yaw_deg * M_PI / 180.0);

      RCLCPP_INFO(
        this->get_logger(),
        "Initial odom pose: x=%.3f y=%.3f yaw=%.1f deg",
        x__, y__, initial_yaw_deg);

      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_deadwheels", 10);
      //tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      subscription_ = this->create_subscription<custom_msgs::msg::DeadwheelTicks>(
      "deadwheel_ticks", rclcpp::SensorDataQoS().keep_last(1), [this](custom_msgs::msg::DeadwheelTicks::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);

        const rclcpp::Time receive_stamp = this->now();
        rclcpp::Time source_stamp;
        bool source_stamp_is_ros_synced{false};
        if (!getMessageStamp(msg, receive_stamp, source_stamp, source_stamp_is_ros_synced)) {
          return;
        }
        rclcpp::Time odom_stamp = source_stamp_is_ros_synced ? source_stamp : receive_stamp;

        if (!initialized_) {
          prevTicks[0] = msg->t0;
          prevTicks[1] = msg->t1;
          prevTicks[2] = msg->t2;
          prev_stamp_ = odom_stamp;
          prev_receive_stamp_ = receive_stamp;
          initialized_ = true;
          publishOdom(odom_stamp);
          return;
        }

        double dt = (odom_stamp - prev_stamp_).seconds();
        const double receive_dt = (receive_stamp - prev_receive_stamp_).seconds();

        if ((dt <= 1e-6 || !std::isfinite(dt)) && receive_dt > 1e-6 && std::isfinite(receive_dt)) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "deadwheel_ticks source dt %.9f is invalid/repeated; using receive dt %.9f",
            dt,
            receive_dt);
          dt = receive_dt;
          odom_stamp = receive_stamp;
        }

        if (dt <= 1e-6 || !std::isfinite(dt)) {
          RCLCPP_WARN(this->get_logger(), "Bad dt (%.9f), skipping", dt);
          return;
        }

        prev_stamp_ = odom_stamp;
        prev_receive_stamp_ = receive_stamp;

        // calculs des delta ticks
        int64_t ticks0 = (msg->t0); 
        int64_t ticks1 = (msg->t1);
        int64_t ticks2 = (msg->t2);
        int64_t dRTicks = ticks0 - prevTicks[0];
        int64_t dLTicks = ticks1 - prevTicks[1];
        int64_t dSTicks = ticks2 - prevTicks[2];

        if (std::llabs(dRTicks) > max_tick_delta_ ||
            std::llabs(dLTicks) > max_tick_delta_ ||
            std::llabs(dSTicks) > max_tick_delta_) {
          RCLCPP_WARN(
            this->get_logger(),
            "Deadwheel tick jump/reset detected; rebasing prev ticks and skipping integration. "
            "delta=[%lld,%lld,%lld] limit=%lld",
            static_cast<long long>(dRTicks),
            static_cast<long long>(dLTicks),
            static_cast<long long>(dSTicks),
            static_cast<long long>(max_tick_delta_));
          prevTicks[0] = ticks0;
          prevTicks[1] = ticks1;
          prevTicks[2] = ticks2;
          vx = 0.0;
          vy = 0.0;
          omega = 0.0;
          publishOdom(odom_stamp);
          return;
        }

        prevTicks[0] = ticks0;
        prevTicks[1] = ticks1;
        prevTicks[2] = ticks2;

        //calculs des déplacements selon les axes du robot
        double rightDist = RIGHT_DEADWHEEL_SIGN * static_cast<double>(dRTicks) * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[0];
        double leftDist = LEFT_DEADWHEEL_SIGN * static_cast<double>(dLTicks) * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[1];
        double dxr = 0.5 * (rightDist + leftDist);
        double dangle = (leftDist - rightDist) / DEADWHEEL_DISTANCE;
        double dyr = (SIDE_DEADWHEEL_SIGN * static_cast<double>(dSTicks) * SDEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[2]) + OFFSET * dangle;
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
        omega = dangle/dt;
        vy = dyr/dt;

        if (stamp_diagnostics_) {
          const double stamp_offset = (receive_stamp - source_stamp).seconds();
          RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "stamp dt=%.4f receive_dt=%.4f jitter=%.4f offset=%.4f synced=%d ticks=[%lld,%lld,%lld] v=[%.3f,%.3f,%.3f]",
            dt,
            receive_dt,
            receive_dt - dt,
            stamp_offset,
            source_stamp_is_ros_synced ? 1 : 0,
            static_cast<long long>(dRTicks),
            static_cast<long long>(dLTicks),
            static_cast<long long>(dSTicks),
            vx,
            vy,
            omega);
        }

        RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 500,  // milliseconds
        "x=%.3f y=%.3f", 
        x__, y__
        );

        publishOdom(odom_stamp);
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
