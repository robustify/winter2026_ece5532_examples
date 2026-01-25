#include <rclcpp/rclcpp.hpp>

namespace ros_time_example {

  class TimeStampExample : public rclcpp::Node {
    public:
      TimeStampExample()
      : rclcpp::Node("time_stamp_example")
      {
      }

    private:
  };

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_instance = std::make_shared<ros_time_example::TimeStampExample>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}
