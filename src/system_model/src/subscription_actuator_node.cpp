#include "system_model/subscription_actuator_node.hpp"

SubscriptionActuatorNode::SubscriptionActuatorNode(
    const rclcpp::NodeOptions &options)
    : Node(options.arguments()[0], options) {
  subscription = this->create_subscription<std_msgs::msg::Header>(
      options.arguments()[2], stoi(options.arguments()[3]),
      [this](std_msgs::msg::Header msg) { subscription_callback(msg); });
  this->wcet = stol(options.arguments()[1]);
}

void SubscriptionActuatorNode::subscription_callback(
    const std_msgs::msg::Header msg) {

  long now = this->get_clock()->now().nanoseconds();
  while (this->get_clock()->now().nanoseconds() < now + wcet) {
  }
  std::cout << "Received message: frame_id \"" << msg.frame_id << "\" stamp \""
            << msg.stamp.sec << " " << msg.stamp.nanosec << "\"" << std::endl;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SubscriptionActuatorNode)