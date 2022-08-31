#include "system_model/filter_node.hpp"

FilterNode::FilterNode(const rclcpp::NodeOptions &options)
    : Node(options.arguments()[0], options) {
  subscription = this->create_subscription<std_msgs::msg::Header>(
      options.arguments()[2], stoi(options.arguments()[4]),
      [this](std_msgs::msg::Header msg) { subscription_callback(msg); });

  publisher = this->create_publisher<std_msgs::msg::Header>(
      options.arguments()[3], stoi(options.arguments()[5]));
  this->wcet = stol(options.arguments()[1]);
}

void FilterNode::subscription_callback(const std_msgs::msg::Header msg) {

  long now = this->get_clock()->now().nanoseconds();
  while (this->get_clock()->now().nanoseconds() < now + wcet) {
  }
  std::cout << "Received message: frame_id \"" << msg.frame_id << "\" stamp \""
            << msg.stamp.sec << " " << msg.stamp.nanosec << "\"" << std::endl;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(FilterNode)