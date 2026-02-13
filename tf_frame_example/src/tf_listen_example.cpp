#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>

namespace tf_frame_example {

  class TfListenExample : public rclcpp::Node {
    public:
      TfListenExample()
      : rclcpp::Node("tf_listen_example")
      {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1000), std::bind(&TfListenExample::timer_callback, this));
        parent_frame_ = declare_parameter<std::string>("parent_frame", "frame1");
        child_frame_ = declare_parameter<std::string>("child_frame", "frame2");
        param_cb_ = add_on_set_parameters_callback(std::bind(&TfListenExample::param_update, this, std::placeholders::_1));
      }

    private:
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      rclcpp::TimerBase::SharedPtr timer_;
      std::string parent_frame_;
      std::string child_frame_;
      void timer_callback() {
        try {
          geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(parent_frame_, child_frame_, rclcpp::Time());

          tf2::Quaternion quat(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
          double yaw_angle, pitch_angle, roll_angle;
          tf2::Matrix3x3(quat).getEulerYPR(yaw_angle, pitch_angle, roll_angle);

          RCLCPP_INFO_STREAM(this->get_logger(), "\nFound transform from " << parent_frame_ << " to " << child_frame_ << ":\nXYZ: "
            << t.transform.translation.x << " " << t.transform.translation.y << " " << t.transform.translation.z
            << "\nRoll: " << roll_angle
            << "\nPitch: " << pitch_angle
            << "\nYaw: " << yaw_angle);
        } catch (tf2::TransformException &ex) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find transform from " << parent_frame_ << " to " << child_frame_ << ": " << ex.what());
        }
      }

      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
      rcl_interfaces::msg::SetParametersResult param_update(const std::vector<rclcpp::Parameter>& parameters) {
        for (const rclcpp::Parameter& p : parameters) {
          if (p.get_name() == "parent_frame") {
            parent_frame_ = p.as_string();
          } else if (p.get_name() == "child_frame") {
            child_frame_ = p.as_string();
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
  auto node_instance = std::make_shared<tf_frame_example::TfListenExample>();
  rclcpp::spin(node_instance);
  rclcpp::shutdown();
  return 0;
}