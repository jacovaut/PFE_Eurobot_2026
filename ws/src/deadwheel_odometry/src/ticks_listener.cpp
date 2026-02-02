#include <rclcpp/rclcpp.hpp>
#include "deadwheel_msgs/msg/deadwheel_ticks.hpp"
#include <cmath>

double x_{0.0}, y_{0.0}, theta_{0.0};

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

    double prevTicks[3] = {0, 0, 0};
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
          prevTicks[0] = 0;
          prevTicks[1] = 0;
          prevTicks[2] = 0;
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
        double ticks0 = (static_cast<double>(msg->t0)); 
        double ticks1 = (static_cast<double>(msg->t1));
        double ticks2 = (static_cast<double>(msg->t2));
        double dRTicks = ticks0 - prevTicks[0];
        double dLTicks = ticks1 - prevTicks[1];
        double dSTicks = ticks2 - prevTicks[2];
        prevTicks[0] = ticks0;
        prevTicks[1] = ticks1;
        prevTicks[2] = ticks2;

        //calculs des déplacements selon les axes du robot
        double rightDist = dRTicks * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[0];
        double leftDist = dLTicks * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[1];
        double dyr = 0.5 * (rightDist + leftDist);
        double dangle = (rightDist - leftDist) / DEADWHEEL_DISTANCE;
        double dxr = (dSTicks * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[2]) - OFFSET * dangle;
        double avgangle = deadwheelodo.angle + dangle/2; 

        //calculs des déplacements selon le world
        double cos = std::cos(avgangle);
        double sin = std::sin(avgangle);
        double dx = dyr*cos - dxr*sin;
        double dy = dyr*sin + dxr*cos;

        //mise à jour de la position du robot
        x_ += dx;
        y_ += dy;
        theta_ = AngleUtils.normalizeRadians(theta_ + dangle);

        //calculs de vitesse
        double vx = dx/dt;
        double vy = dy/dt;
        double omega = dangle/dt;

        RCLCPP_INFO(
        this->get_logger(),
        "x=%.3f y=%.3f th=%.3f | vx=%.3f vy=%.3f w=%.3f",
        x_, y_, theta_, vx, vy, omega
       );
      }
    ); 
  };
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TicksListener>());
  rclcpp::shutdown();
  return 0;
}

