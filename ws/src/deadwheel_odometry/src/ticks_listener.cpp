#include <rclcpp/rclcpp.hpp>
#include "deadwheel_msgs/msg/deadwheel_ticks.hpp"
#include <cmath>

double g_x{0.0}, g_y{0.0}, g_theta{0.0};

double normalizeAngleSigned(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

class TicksListener : public rclcpp::Node
{
  private : 
    rclcpp::Subscription<deadwheel_msgs::msg::DeadwheelTicks>::SharedPtr subscription_;
  
    //Constantes, à ajouter les bonnes valeurs
    const double ENCODER_TICKS_PER_REVOLUTION [3] = {1000, 1000, 1000};
    const double DEADWHEEL_DIAMETER  = 2;
    const double DEADWHEEL_CIRCUMFERENCE = (M_PI) * DEADWHEEL_DIAMETER;
    const double DEADWHEEL_DISTANCE = 10; //distance entre les deux deadwheel principaux
    const double OFFSET = 2; //distance entre le side deadwheel et le centre de rotation du robot

    int64_t prevTicks[3] = {0, 0, 0};
    bool initialized_{false};
    rclcpp::Time prev_stamp_;

  public:
    TicksListener() : Node("ticks_listener")
    {
      subscription_ = this->create_subscription<deadwheel_msgs::msg::DeadwheelTicks>(
      "deadwheel_ticks", 10,[this](deadwheel_msgs::msg::DeadwheelTicks::SharedPtr msg)
      {
        const rclcpp::Time stamp(msg->header.stamp);

        if (!initialized_) {
          prevTicks[0] = msg->t0;
          prevTicks[1] = msg->t1;
          prevTicks[2] = msg->t2;
          prev_stamp_ = stamp;
          initialized_ = true;
         return;
        }

        double dt = (stamp - prev_stamp_).seconds();
        prev_stamp_ = stamp;

        if (dt <= 1e-6) {
          RCLCPP_WARN(this->get_logger(), "dt too small (%.9f), skipping", dt);
          return;
        }

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
        double avgangle = g_theta + dangle/2; 

        //calculs des déplacements selon le world
        double c = std::cos(avgangle);
        double s = std::sin(avgangle);
        double dx = dxr*c - dyr*s;
        double dy = dyr*c + dxr*s;

        //mise à jour de la position du robot
        g_x += dx;
        g_y += dy;
        g_theta = normalizeAngleSigned(g_theta + dangle);

        //calculs de vitesse
        double vx = dx/dt;
        double vy = dy/dt;
        double omega = dangle/dt;

        RCLCPP_INFO(
        this->get_logger(),
        "x=%.3f y=%.3f th=%.3f | vx=%.3f vy=%.3f w=%.3f",
        g_x, g_y, g_theta, vx, vy, omega
       );
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

