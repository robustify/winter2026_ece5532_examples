#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "enu.hpp"

namespace gps_example {
  class GpsExample : public rclcpp::Node {
    public:
      GpsExample()
      : rclcpp::Node("gps_example")
      {
        double ref_lat = declare_parameter<double>("ref_lat", INFINITY);
        double ref_lon = declare_parameter<double>("ref_lon", INFINITY);
        double ref_alt = declare_parameter<double>("ref_alt", INFINITY);
        if (!std::isfinite(ref_lat) || !std::isfinite(ref_lon) || !std::isfinite(ref_alt)) {
          RCLCPP_ERROR(get_logger(), "Reference coordinate parameters not set!");
          rclcpp::shutdown();
          return;
        }

        // TODO: Subscribe to GPS position and advertise an ENU path

        // TODO: Calculate ECEF position and ECEF -> ENU rotation matrix at the reference coordinates
      }

    private:

      void recv_fix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        // TODO: Convert incoming latitude/longitude/altitude data into ENU

        // TODO: Append position to the end of the path message and then publish it out
      }
  };
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_instance = std::make_shared<gps_example::GpsExample>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}
