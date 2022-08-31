#include "system_model/sensor_node.hpp"
#include <cstdlib>
#include <rclcpp/node_options.hpp>

SensorNode::SensorNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node(options.arguments()[0], options) {

  timer = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { timer_callback(); });
  publisher = this->create_publisher<std_msgs::msg::Header>("scan", 1);
  this->wcet = 1000;

  // timer = this->create_wall_timer(
  //     std::chrono::milliseconds(stoi(options.arguments()[1])),
  //     [this]() { timer_callback(); });
  // publisher = this->create_publisher<std_msgs::msg::Header>(
  //     options.arguments()[3], stoi(options.arguments()[4]));
  // this->wcet = stol(options.arguments()[2]);
}

void SensorNode::timer_callback() {
  long now = this->get_clock()->now().nanoseconds();
  while (this->get_clock()->now().nanoseconds() < now + wcet) {
  }
  std_msgs::msg::Header msg;
  msg.stamp = this->get_clock()->now();
  publisher->publish(msg);
  std::cout << "Sent message: frame_id \"" << msg.frame_id << "\" stamp \""
            << msg.stamp.sec << " " << msg.stamp.nanosec << "\"" << std::endl;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SensorNode)