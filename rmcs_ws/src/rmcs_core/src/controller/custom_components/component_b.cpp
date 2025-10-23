#include "rclcpp/rclcpp.hpp"
#include "rmcs_msgs/msg/double_stamped.hpp"

class ComponentB : public rclcpp::Node {
public:
  ComponentB() : Node("component_b") {
    // 订阅ComponentA的DoubleStamped输出
    sub_sin_ = this->create_subscription<rmcs_msgs::msg::DoubleStamped>(
      "sin_output", 10,
      std::bind(&ComponentB::sin_cb, this, std::placeholders::_1)
    );
    sub_cos_ = this->create_subscription<rmcs_msgs::msg::DoubleStamped>(
      "cos_output", 10,
      std::bind(&ComponentB::cos_cb, this, std::placeholders::_1)
    );

    // 用DoubleStamped发布求和结果
    pub_sum_ = this->create_publisher<rmcs_msgs::msg::DoubleStamped>("sum_output", 10);
  }

private:
  void sin_cb(const rmcs_msgs::msg::DoubleStamped::SharedPtr msg) {
    sin_val_ = msg->data;
    pub_sum();
  }
  void cos_cb(const rmcs_msgs::msg::DoubleStamped::SharedPtr msg) {
    cos_val_ = msg->data;
    pub_sum();
  }
  void pub_sum() {
    auto sum_msg = rmcs_msgs::msg::DoubleStamped();
    sum_msg.header.stamp = this->now();
    sum_msg.data = sin_val_ + cos_val_;
    pub_sum_->publish(sum_msg);
  }

  rclcpp::Subscription<rmcs_msgs::msg::DoubleStamped>::SharedPtr sub_sin_;
  rclcpp::Subscription<rmcs_msgs::msg::DoubleStamped>::SharedPtr sub_cos_;
  rclcpp::Publisher<rmcs_msgs::msg::DoubleStamped>::SharedPtr pub_sum_;
  double sin_val_ = 0.0;
  double cos_val_ = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComponentB>());
  rclcpp::shutdown();
  return 0;
}