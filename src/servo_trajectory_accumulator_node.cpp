#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class ServoTrajectoryAccumulator : public rclcpp::Node
{
public:
  ServoTrajectoryAccumulator()
  : Node("servo_trajectory_accumulator")
  {
    declare_parameter("input_topic", "/servo_node/joint_trajectory_raw");
    declare_parameter("output_topic", "/fr3_arm_controller/joint_trajectory");
    declare_parameter("controller_state_topic", "/fr3_arm_controller/controller_state");
    declare_parameter(
      "flange_roll_velocity_topic",
      "/phantom_panda_teleop_node/flange_roll_velocity");
    declare_parameter("publish_rate", 100.0);
    declare_parameter("input_timeout", 0.05);
    declare_parameter("controller_state_timeout", 0.10);
    declare_parameter<std::vector<double>>(
      "max_joint_velocity", {0.20, 0.20, 0.20, 0.20, 0.30, 0.30, 0.30});
    declare_parameter<std::vector<double>>(
      "max_joint_acceleration", {0.50, 0.50, 0.50, 0.50, 1.00, 1.00, 1.00});
    declare_parameter<std::vector<double>>(
      "max_joint_jerk", {5.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0});

    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("controller_state_topic", controller_state_topic_);
    get_parameter("flange_roll_velocity_topic", flange_roll_velocity_topic_);
    get_parameter("publish_rate", publish_rate_);
    get_parameter("input_timeout", input_timeout_);
    get_parameter("controller_state_timeout", controller_state_timeout_);
    get_parameter("max_joint_velocity", max_joint_velocity_);
    get_parameter("max_joint_acceleration", max_joint_acceleration_);
    get_parameter("max_joint_jerk", max_joint_jerk_);

    if (publish_rate_ <= 0.0 || input_timeout_ <= 0.0 || controller_state_timeout_ <= 0.0 ||
      max_joint_velocity_.size() != kJointCount ||
      max_joint_acceleration_.size() != kJointCount ||
      max_joint_jerk_.size() != kJointCount)
    {
      throw std::invalid_argument(
              "Rates/timeouts must be positive and joint limits must have seven values");
    }
    for (size_t i = 0; i < kJointCount; ++i) {
      if (!std::isfinite(max_joint_velocity_[i]) || max_joint_velocity_[i] <= 0.0 ||
        !std::isfinite(max_joint_acceleration_[i]) || max_joint_acceleration_[i] <= 0.0 ||
        !std::isfinite(max_joint_jerk_[i]) || max_joint_jerk_[i] <= 0.0)
      {
        throw std::invalid_argument("Accumulator limits must be finite and positive");
      }
    }

    output_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(output_topic_, 10);
    input_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      input_topic_, 10,
      std::bind(&ServoTrajectoryAccumulator::input_callback, this, std::placeholders::_1));
    state_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      controller_state_topic_, 10,
      std::bind(&ServoTrajectoryAccumulator::state_callback, this, std::placeholders::_1));
    flange_roll_sub_ = create_subscription<std_msgs::msg::Float64>(
      flange_roll_velocity_topic_, 10,
      std::bind(&ServoTrajectoryAccumulator::flange_roll_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ServoTrajectoryAccumulator::publish_reference, this));

    RCLCPP_INFO(
      get_logger(), "Rate-limited Servo velocity bridge: %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  static constexpr size_t kJointCount = 7;

  static std::string joint_name(size_t index)
  {
    return "fr3_joint" + std::to_string(index + 1);
  }

  template<typename NamesT>
  static bool canonical_indices(
    const NamesT & names, std::array<size_t, kJointCount> & indices)
  {
    for (size_t i = 0; i < kJointCount; ++i) {
      const auto it = std::find(names.begin(), names.end(), joint_name(i));
      if (it == names.end()) {
        return false;
      }
      indices[i] = static_cast<size_t>(std::distance(names.begin(), it));
    }
    return true;
  }

  void state_callback(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    std::array<size_t, kJointCount> indices{};
    if (!canonical_indices(msg->joint_names, indices) ||
      msg->feedback.positions.size() < msg->joint_names.size())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring incomplete arm controller feedback");
      return;
    }

    for (size_t i = 0; i < kJointCount; ++i) {
      const double position = msg->feedback.positions[indices[i]];
      if (!std::isfinite(position)) {
        return;
      }
      measured_positions_[i] = position;
    }
    have_state_ = true;
    last_state_time_ = now();
    if (!filter_initialized_) {
      applied_velocities_.fill(0.0);
      applied_accelerations_.fill(0.0);
      filter_initialized_ = true;
      last_update_time_ = last_state_time_;
    }
  }

  void input_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      return;
    }
    std::array<size_t, kJointCount> indices{};
    if (!canonical_indices(msg->joint_names, indices) ||
      msg->points.front().velocities.size() < msg->joint_names.size())
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Servo raw trajectory must contain velocities for all seven FR3 joints");
      have_input_ = false;
      return;
    }

    for (size_t i = 0; i < kJointCount; ++i) {
      const double velocity = msg->points.front().velocities[indices[i]];
      if (!std::isfinite(velocity)) {
        have_input_ = false;
        return;
      }
      requested_velocities_[i] = std::clamp(
        velocity, -max_joint_velocity_[i], max_joint_velocity_[i]);
    }
    last_input_time_ = now();
    have_input_ = true;
  }

  void flange_roll_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      have_flange_roll_command_ = false;
      return;
    }
    flange_roll_velocity_ = std::clamp(
      msg->data, -max_joint_velocity_[kJointCount - 1], max_joint_velocity_[kJointCount - 1]);
    last_flange_roll_command_time_ = now();
    have_flange_roll_command_ = true;
  }

  void publish_reference()
  {
    if (!have_state_ || !filter_initialized_) {
      return;
    }

    const rclcpp::Time current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    dt = std::clamp(dt, 0.0, 2.0 / publish_rate_);

    const bool state_fresh =
      (current_time - last_state_time_).seconds() <= controller_state_timeout_;
    const bool input_fresh = state_fresh && have_input_ &&
      (current_time - last_input_time_).seconds() <= input_timeout_;
    std::array<double, kJointCount> target_velocities{};
    if (input_fresh) {
      target_velocities = requested_velocities_;
    } else if (!state_fresh) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Arm controller state is stale; ramping the joint velocity command to zero");
    } else if (have_input_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Servo raw trajectory is stale; ramping the joint velocity command to zero");
    }

    // Stylus roll is a joint-space command, deliberately bypassing Cartesian IK.
    // It overrides only fr3_joint7; joints 1--6 retain the Servo command (or zero).
    const bool flange_roll_fresh = state_fresh && have_flange_roll_command_ &&
      (current_time - last_flange_roll_command_time_).seconds() <= input_timeout_;
    if (flange_roll_fresh) {
      target_velocities[kJointCount - 1] = flange_roll_velocity_;
    }

    for (size_t i = 0; i < kJointCount; ++i) {
      if (dt > 0.0) {
        const double velocity_error = target_velocities[i] - applied_velocities_[i];
        const double target_acceleration = std::clamp(
          velocity_error / dt, -max_joint_acceleration_[i], max_joint_acceleration_[i]);
        const double max_acceleration_change = max_joint_jerk_[i] * dt;
        applied_accelerations_[i] = std::clamp(
          target_acceleration,
          applied_accelerations_[i] - max_acceleration_change,
          applied_accelerations_[i] + max_acceleration_change);
        // Never retain acceleration that points away from a newly changed target.
        // This matters especially when Servo halts or reverses a command: preserving
        // the old acceleration merely to satisfy a jerk bound would increase speed
        // before braking starts.
        if (velocity_error * applied_accelerations_[i] < 0.0) {
          applied_accelerations_[i] = 0.0;
        }

        const double next_velocity =
          applied_velocities_[i] + applied_accelerations_[i] * dt;
        const bool crosses_target =
          (velocity_error > 0.0 && next_velocity > target_velocities[i]) ||
          (velocity_error < 0.0 && next_velocity < target_velocities[i]);
        if (crosses_target || velocity_error == 0.0) {
          applied_velocities_[i] = target_velocities[i];
          applied_accelerations_[i] = 0.0;
        } else {
          applied_velocities_[i] = std::clamp(
            next_velocity, -max_joint_velocity_[i], max_joint_velocity_[i]);
        }
      }
    }

    trajectory_msgs::msg::JointTrajectory output;
    output.header.stamp = rclcpp::Time(0);
    output.header.frame_id = "fr3_link0";
    output.points.resize(1);
    // With the controller's `interpolation_method: none`, a zero-time point is
    // applied immediately. This prevents the cubic spline overshoot observed when
    // rapidly replacing 10 ms position+velocity trajectories on real hardware.
    output.points.front().time_from_start = rclcpp::Duration::from_seconds(0.0);
    for (size_t i = 0; i < kJointCount; ++i) {
      output.joint_names.push_back(joint_name(i));
      output.points.front().positions.push_back(measured_positions_[i]);
      output.points.front().velocities.push_back(applied_velocities_[i]);
    }
    output_pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string controller_state_topic_;
  std::string flange_roll_velocity_topic_;
  double publish_rate_;
  double input_timeout_;
  double controller_state_timeout_;
  std::vector<double> max_joint_velocity_;
  std::vector<double> max_joint_acceleration_;
  std::vector<double> max_joint_jerk_;
  std::array<double, kJointCount> measured_positions_{};
  std::array<double, kJointCount> requested_velocities_{};
  std::array<double, kJointCount> applied_velocities_{};
  std::array<double, kJointCount> applied_accelerations_{};
  bool have_state_{false};
  bool have_input_{false};
  bool filter_initialized_{false};
  bool have_flange_roll_command_{false};
  double flange_roll_velocity_{0.0};
  rclcpp::Time last_state_time_;
  rclcpp::Time last_input_time_;
  rclcpp::Time last_update_time_;
  rclcpp::Time last_flange_roll_command_time_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr input_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr flange_roll_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoTrajectoryAccumulator>());
  rclcpp::shutdown();
  return 0;
}
