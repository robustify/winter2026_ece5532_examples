#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace marker_example {

  class TfBroadcaster : public rclcpp::Node {
    public:
      TfBroadcaster()
      : rclcpp::Node("tf_broadcaster")
      {
        // Timer to update TF frame periodically
        timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20), std::bind(&TfBroadcaster::timer_callback, this));

        // Initialize a TF broadcaster instance
        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create stamped transform message property and set the parent and child frame IDs
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = "frame1";

        // Declare parameters for 2-D position and angle, and bind an update callback
        declare_parameter<double>("x", 0.0);
        declare_parameter<double>("y", 0.0);
        declare_parameter<double>("yaw", 0.0);
        param_cb = add_on_set_parameters_callback(std::bind(&TfBroadcaster::param_update, this, std::placeholders::_1));
      }

    private:

      rclcpp::TimerBase::SharedPtr timer;
      std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
      geometry_msgs::msg::TransformStamped transform_msg;

      void timer_callback() {
        // Update the timestamp in the stamped transform header and update the TF frame using the broadcaster
        transform_msg.header.stamp = get_clock()->now();
        broadcaster->sendTransform(transform_msg);
      }

      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb;
      rcl_interfaces::msg::SetParametersResult param_update(const std::vector<rclcpp::Parameter>& parameters) {
        // Update values in stamped transform with the latest parameter values

        for (const rclcpp::Parameter& p : parameters) {
          if (p.get_name() == "x") {
            transform_msg.transform.translation.x = p.as_double();
          } else if (p.get_name() == "y") {
            transform_msg.transform.translation.y = p.as_double();
          } else if (p.get_name() == "yaw") {
            tf2::Quaternion q;
            q.setRPY(0, 0, p.as_double());
            // transform_msg.transform.rotation.w = q.w();
            // transform_msg.transform.rotation.x = q.x();
            // transform_msg.transform.rotation.y = q.y();
            // transform_msg.transform.rotation.z = q.z();
            tf2::convert(q, transform_msg.transform.rotation);
          }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      }
  };
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_instance = std::make_shared<marker_example::TfBroadcaster>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}
