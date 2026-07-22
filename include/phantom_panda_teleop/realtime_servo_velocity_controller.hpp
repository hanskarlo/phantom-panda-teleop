#pragma once

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace phantom_panda_teleop
{

class RealtimeServoVelocityController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  static constexpr std::size_t kJointCount = 7;

  struct VelocityCommand
  {
    std::array<double, kJointCount> velocities{};
    std::int64_t receipt_time_ns{0};
    bool valid{false};
  };

  void command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  bool configure_state_message();
  void publish_state(const rclcpp::Time & time);

  std::vector<std::string> joints_;
  double command_timeout_{0.05};
  double state_publish_rate_{50.0};
  std::array<double, kJointCount> max_velocity_{};
  std::array<double, kJointCount> max_acceleration_{};
  std::array<double, kJointCount> max_jerk_{};

  realtime_tools::RealtimeBuffer<VelocityCommand> command_buffer_;
  std::array<double, kJointCount> applied_velocity_{};
  std::array<double, kJointCount> applied_acceleration_{};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_subscriber_;
  using StateMessage = control_msgs::msg::JointTrajectoryControllerState;
  std::shared_ptr<realtime_tools::RealtimePublisher<StateMessage>> state_publisher_;
  rclcpp::Duration state_publish_period_{0, 0};
  rclcpp::Time last_state_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace phantom_panda_teleop
