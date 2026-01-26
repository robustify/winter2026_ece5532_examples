#include <rclcpp/rclcpp.hpp>

namespace ros_time_example {

  class TimerExample : public rclcpp::Node {
    public:
      TimerExample()
      : rclcpp::Node("timer_example")
      {
        timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1000),
          std::bind(&TimerExample::timer_callback, this));
      }

    private:
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Time last_time_;

      void timer_callback() {
        if (last_time_.get_clock_type() != get_clock()->now().get_clock_type()) {
          last_time_ = get_clock()->now();
          return;
        }

        rclcpp::Time current_time = get_clock()->now();
        rclcpp::Duration time_diff = current_time - last_time_;
        last_time_ = current_time;

        RCLCPP_INFO_STREAM(get_logger(), "Time since last trigger: " << time_diff.seconds());
      }
  };
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_instance = std::make_shared<ros_time_example::TimerExample>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}
