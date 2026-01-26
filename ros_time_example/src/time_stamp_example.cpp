#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace ros_time_example {

  class TimeStampExample : public rclcpp::Node {
    public:
      TimeStampExample()
      : rclcpp::Node("time_stamp_example")
      {
        sub_twist_ = create_subscription<geometry_msgs::msg::Twist>("twist", 1, std::bind(&TimeStampExample::recv_twist, this, std::placeholders::_1));
        pub_twist_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist_stamped", 1);
      }

    private:
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_stamped_;

      void recv_twist(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        geometry_msgs::msg::TwistStamped twist_stamped_msg;

        twist_stamped_msg.header.stamp = get_clock()->now();
        twist_stamped_msg.twist = *msg;

        pub_twist_stamped_->publish(twist_stamped_msg);
      }

  };

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_instance = std::make_shared<ros_time_example::TimeStampExample>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}
