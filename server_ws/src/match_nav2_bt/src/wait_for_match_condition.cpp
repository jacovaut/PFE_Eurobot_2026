#include "match_nav2_bt/wait_for_match_condition.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace match_nav2_bt
{

WaitForMatch::WaitForMatch(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  if (!getInput("topic", topic_)) {
    topic_ = "/match/running";
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  match_running_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_,
    qos,
    std::bind(&WaitForMatch::onMatchRunning, this, std::placeholders::_1),
    options);
}

BT::NodeStatus WaitForMatch::onStart()
{
  return onRunning();
}

BT::NodeStatus WaitForMatch::onRunning()
{
  callback_group_executor_.spin_some();

  if (match_running_.load()) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForMatch::onHalted()
{
}

void WaitForMatch::onMatchRunning(const std_msgs::msg::Bool::SharedPtr msg)
{
  match_running_.store(msg->data);
}

}  // namespace match_nav2_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<match_nav2_bt::WaitForMatch>("WaitForMatch");
}
