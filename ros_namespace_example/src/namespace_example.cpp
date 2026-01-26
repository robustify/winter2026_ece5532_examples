#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros_namespace_example {

  class NamespaceExample : public rclcpp::Node {
    public:
      NamespaceExample()
      : rclcpp::Node("namespace_example")
      {
        sub_local_topic_ = create_subscription<std_msgs::msg::String>("local_sub_topic", 1, std::bind(&NamespaceExample::recv_local, this, std::placeholders::_1));
        sub_global_topic_ = create_subscription<std_msgs::msg::String>("/global_sub_topic", 1, std::bind(&NamespaceExample::recv_global, this, std::placeholders::_1));
        pub_local_topic_ = create_publisher<std_msgs::msg::String>("local_pub_topic", 1);
        pub_global_topic_ = create_publisher<std_msgs::msg::String>("/global_pub_topic", 1);
      }

    private:
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_global_topic_;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_local_topic_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_global_topic_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_local_topic_;

      void recv_local(const std_msgs::msg::String::ConstSharedPtr msg) {

      }

      void recv_global(const std_msgs::msg::String::ConstSharedPtr msg) {

      }
  };
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_instance = std::make_shared<ros_namespace_example::NamespaceExample>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}
