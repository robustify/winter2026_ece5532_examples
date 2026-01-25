#include <rclcpp/rclcpp.hpp>

namespace ros_namespace_example {

  class NamespaceExample : public rclcpp::Node {
    public:
      NamespaceExample()
      : rclcpp::Node("namespace_example")
      {
      }

    private:
  };
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_instance = std::make_shared<ros_namespace_example::NamespaceExample>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}
