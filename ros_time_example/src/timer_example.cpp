#include <rclcpp/rclcpp.hpp>

namespace ros_time_example {

  class TimerExample : public rclcpp::Node {
    public:
      TimerExample()
      : rclcpp::Node("timer_example")
      {
      }

    private:
  };
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_instance = std::make_shared<ros_time_example::TimerExample>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}
