#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

class SimHapticNode : public rclcpp::Node
{
public:
  SimHapticNode()
  : Node("sim_haptic"), position_{0.0, 0.0, 0.0}, orientation_(0.0, 0.0, 0.0, 1.0)
  {
    declare_parameter("update_rate", 200.0);
    declare_parameter("command_timeout", 0.15);
    declare_parameter("base_frame", "touch_x_base");
    declare_parameter("ee_frame", "touch_x_ee");
    declare_parameter("joy_topic", "/geomagic_touch_x/joy");
    declare_parameter("twist_topic", "~/twist_cmd");

    update_rate_ = get_parameter("update_rate").as_double();
    command_timeout_ = get_parameter("command_timeout").as_double();
    base_frame_ = get_parameter("base_frame").as_string();
    ee_frame_ = get_parameter("ee_frame").as_string();
    joy_topic_ = get_parameter("joy_topic").as_string();
    const std::string twist_topic = get_parameter("twist_topic").as_string();

    if (update_rate_ <= 0.0 || command_timeout_ <= 0.0) {
      throw std::invalid_argument("Simulated haptic rates and timeouts must be positive.");
    }

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(joy_topic_, rclcpp::SensorDataQoS());
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        commanded_twist_ = msg->twist;
        last_twist_time_ = now();
        have_twist_ = true;
      });

    clutch_service_ = create_service<std_srvs::srv::SetBool>(
      "~/set_clutch",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        clutch_pressed_ = request->data;
        response->success = true;
        response->message = clutch_pressed_ ? "Sim clutch engaged." : "Sim clutch released.";
      });
    button2_service_ = create_service<std_srvs::srv::SetBool>(
      "~/set_button2",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        button2_pressed_ = request->data;
        response->success = true;
        response->message = button2_pressed_ ? "Sim Button 2 pressed." :
        "Sim Button 2 released.";
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_pose",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        position_[0] = position_[1] = position_[2] = 0.0;
        orientation_.setValue(0.0, 0.0, 0.0, 1.0);
        commanded_twist_ = geometry_msgs::msg::Twist();
        have_twist_ = false;
        response->success = true;
        response->message = "Sim haptic pose reset.";
      });

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = create_wall_timer(period, std::bind(&SimHapticNode::update, this));
    last_update_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Simulated haptic ready. Command %s and control the buttons with "
      "~/set_clutch and ~/set_button2.",
      twist_topic.c_str());
  }

private:
  void update()
  {
    const rclcpp::Time current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    dt = std::clamp(dt, 0.0, 2.0 / update_rate_);

    geometry_msgs::msg::Twist active_twist;
    if (have_twist_ && (current_time - last_twist_time_).seconds() <= command_timeout_) {
      active_twist = commanded_twist_;
    }

    position_[0] += active_twist.linear.x * dt;
    position_[1] += active_twist.linear.y * dt;
    position_[2] += active_twist.linear.z * dt;

    tf2::Quaternion angular_increment;
    angular_increment.setRPY(
      active_twist.angular.x * dt,
      active_twist.angular.y * dt,
      active_twist.angular.z * dt);
    orientation_ = orientation_ * angular_increment;
    orientation_.normalize();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = ee_frame_;
    transform.transform.translation.x = position_[0];
    transform.transform.translation.y = position_[1];
    transform.transform.translation.z = position_[2];
    transform.transform.rotation.x = orientation_.x();
    transform.transform.rotation.y = orientation_.y();
    transform.transform.rotation.z = orientation_.z();
    transform.transform.rotation.w = orientation_.w();
    transform_broadcaster_->sendTransform(transform);

    sensor_msgs::msg::Joy joy;
    joy.header.stamp = current_time;
    joy.header.frame_id = ee_frame_;
    joy.buttons = {clutch_pressed_ ? 1 : 0, button2_pressed_ ? 1 : 0};
    joy_pub_->publish(joy);
  }

  double update_rate_;
  double command_timeout_;
  std::string base_frame_;
  std::string ee_frame_;
  std::string joy_topic_;
  double position_[3];
  tf2::Quaternion orientation_;
  geometry_msgs::msg::Twist commanded_twist_;
  rclcpp::Time last_twist_time_;
  rclcpp::Time last_update_time_;
  bool have_twist_{false};
  bool clutch_pressed_{false};
  bool button2_pressed_{false};

  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clutch_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr button2_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimHapticNode>());
  rclcpp::shutdown();
  return 0;
}
