#ifndef MATCH_NAV2_BT__WAIT_FOR_MATCH_CONDITION_HPP_
#define MATCH_NAV2_BT__WAIT_FOR_MATCH_CONDITION_HPP_

#include <atomic>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace match_nav2_bt
{

class WaitForMatch : public BT::StatefulActionNode
{
public:
  WaitForMatch(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "/match/running", "Bool topic that becomes true when robot motion may start")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void onMatchRunning(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr match_running_sub_;
  std::string topic_;
  std::atomic_bool match_running_{false};
};

}  // namespace match_nav2_bt

#endif  // MATCH_NAV2_BT__WAIT_FOR_MATCH_CONDITION_HPP_
