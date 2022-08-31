#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

class SensorNode : public rclcpp::Node {
public:
  SensorNode(const rclcpp::NodeOptions &options);

private:
  void timer_callback();

  long int wcet;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher;
};