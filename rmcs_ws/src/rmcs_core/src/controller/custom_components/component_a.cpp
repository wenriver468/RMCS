#include "rclcpp/rclcpp.hpp"
#include "rmcs_msgs/msg/double_stamped.hpp"
#include <cmath>

class ComponentA : public rclcpp::Node {
public:
  ComponentA() : Node("component_a") {
    this->declare_parameter("omega", 1.0);
    this->get_parameter("omega", omega_);

    // 用DoubleStamped发布sin/cos
    pub_sin_ = this->create_publisher<rmcs_msgs::msg::DoubleStamped>("sin_output", 10);
    pub_cos_ = this->create_publisher<rmcs_msgs::msg::DoubleStamped>("cos_output", 10);

    timer_ = this->create_wall_timer(
      std::chrono::microseconds(1000),  // 1000Hz
      std::bind(&ComponentA::timer_cb, this)
    );
  }

private:
  void timer_cb() {
    auto now = this->now();
    double t = now.seconds();
    
    // 封装sin值到DoubleStamped
    auto sin_msg = rmcs_msgs::msg::DoubleStamped();
    sin_msg.header.stamp = now;
    sin_msg.data = std::sin(omega_ * t);
    pub_sin_->publish(sin_msg);

    // 封装cos值到DoubleStamped
    auto cos_msg = rmcs_msgs::msg::DoubleStamped();
    cos_msg.header.stamp = now;
    cos_msg.data = std::cos(omega_ * t);
    pub_cos_->publish(cos_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rmcs_msgs::msg::DoubleStamped>::SharedPtr pub_sin_;
  rclcpp::Publisher<rmcs_msgs::msg::DoubleStamped>::SharedPtr pub_cos_;
  double omega_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComponentA>());
  rclcpp::shutdown();
  return 0;
}