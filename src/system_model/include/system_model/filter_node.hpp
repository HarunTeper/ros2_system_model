#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

class FilterNode : public rclcpp::Node {
public:
  FilterNode(const rclcpp::NodeOptions &options);

private:
  void subscription_callback(const std_msgs::msg::Header);
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscription;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher;
  long int wcet;
};