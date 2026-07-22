#include "phantom_panda_teleop/realtime_servo_velocity_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <unordered_set>
#include <utility>

namespace phantom_panda_teleop
{

namespace
{

constexpr double kControlPeriod = 1e-3;
constexpr double kLibfrankaMaxJointAcceleration = 10.0 - 1e-3;
constexpr double kLibfrankaMaxJointJerk = 5000.0 - 1e-3;

// Equivalent to libfranka's joint-velocity limitRate overload, but kept local so
// this controller can be built package-only even when a separate libfranka source
// tree is present in the ROS workspace. The FCI joint velocity interface always
// uses a fixed 1 ms command period.
double limit_velocity(
  double upper_velocity, double lower_velocity, double max_acceleration, double max_jerk,
  double target_velocity, double previous_velocity, double previous_acceleration)
{
  const double requested_jerk =
    (((target_velocity - previous_velocity) / kControlPeriod) - previous_acceleration) /
    kControlPeriod;
  const double acceleration = previous_acceleration +
    std::clamp(requested_jerk, -max_jerk, max_jerk) * kControlPeriod;

  const double safe_max_acceleration = std::min(
    (max_jerk / max_acceleration) * (upper_velocity - previous_velocity), max_acceleration);
  const double safe_min_acceleration = std::max(
    (max_jerk / max_acceleration) * (lower_velocity - previous_velocity), -max_acceleration);
  return previous_velocity +
         std::clamp(acceleration, safe_min_acceleration, safe_max_acceleration) * kControlPeriod;
}

}  // namespace

controller_interface::CallbackReturn RealtimeServoVelocityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<double>("command_timeout", 0.05);
    auto_declare<double>("state_publish_rate", 50.0);
    auto_declare<std::vector<double>>(
      "max_joint_velocity", {0.20, 0.20, 0.20, 0.20, 0.30, 0.30, 0.30});
    auto_declare<std::vector<double>>(
      "max_joint_acceleration", {0.50, 0.50, 0.50, 0.50, 1.00, 1.00, 1.00});
    // These are deliberately far below libfranka's 5000 rad/s^3 joint limit,
    // while still allowing the controller to track the already-smoothed 100 Hz input.
    auto_declare<std::vector<double>>(
      "max_joint_jerk", {500.0, 500.0, 500.0, 500.0, 1000.0, 1000.0, 1000.0});
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Controller initialization failed: %s",
      exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RealtimeServoVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  state_publish_rate_ = get_node()->get_parameter("state_publish_rate").as_double();
  const auto velocity_limits =
    get_node()->get_parameter("max_joint_velocity").as_double_array();
  const auto acceleration_limits =
    get_node()->get_parameter("max_joint_acceleration").as_double_array();
  const auto jerk_limits = get_node()->get_parameter("max_joint_jerk").as_double_array();

  if (joints_.size() != kJointCount || velocity_limits.size() != kJointCount ||
    acceleration_limits.size() != kJointCount || jerk_limits.size() != kJointCount ||
    !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
    !std::isfinite(state_publish_rate_) || state_publish_rate_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected seven joints and seven finite velocity/acceleration/jerk limits; rates must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!std::isfinite(velocity_limits[index]) || velocity_limits[index] <= 0.0 ||
      !std::isfinite(acceleration_limits[index]) || acceleration_limits[index] <= 0.0 ||
      !std::isfinite(jerk_limits[index]) || jerk_limits[index] <= 0.0 ||
      acceleration_limits[index] >= kLibfrankaMaxJointAcceleration ||
      jerk_limits[index] >= kLibfrankaMaxJointJerk)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joint limits must be finite, positive, and strictly below libfranka limits");
      return controller_interface::CallbackReturn::ERROR;
    }
    max_velocity_[index] = velocity_limits[index];
    max_acceleration_[index] = acceleration_limits[index];
    max_jerk_[index] = jerk_limits[index];
  }

  const std::unordered_set<std::string> unique_joints(joints_.begin(), joints_.end());
  if (unique_joints.size() != joints_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint names must be unique");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
    std::bind(&RealtimeServoVelocityController::command_callback, this, std::placeholders::_1));

  const auto publisher = get_node()->create_publisher<StateMessage>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());
  state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<StateMessage>>(publisher);
  if (!configure_state_message()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  state_publish_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate_);
  command_buffer_.initRT(VelocityCommand{});

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured 1 kHz Franka velocity rate limiter; input timeout %.3f s",
    command_timeout_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RealtimeServoVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/velocity");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
RealtimeServoVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/position");
    configuration.names.push_back(joint + "/velocity");
  }
  return configuration;
}

controller_interface::CallbackReturn RealtimeServoVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != kJointCount || state_interfaces_.size() != 2 * kJointCount) {
    RCLCPP_ERROR(get_node()->get_logger(), "Required joint interfaces were not assigned");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_buffer_.initRT(VelocityCommand{});
  applied_velocity_.fill(0.0);
  applied_acceleration_.fill(0.0);
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  last_state_publish_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RealtimeServoVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_buffer_.initRT(VelocityCommand{});
  applied_velocity_.fill(0.0);
  applied_acceleration_.fill(0.0);
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void RealtimeServoVelocityController::command_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Ignoring empty velocity trajectory");
    return;
  }

  const auto & point = msg->points.front();
  if (point.velocities.size() != msg->joint_names.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Velocity trajectory has inconsistent joint and velocity array sizes");
    return;
  }

  VelocityCommand command;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const auto name = std::find(msg->joint_names.begin(), msg->joint_names.end(), joints_[index]);
    if (name == msg->joint_names.end()) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Velocity trajectory is missing joint %s", joints_[index].c_str());
      return;
    }
    const auto source_index = static_cast<std::size_t>(
      std::distance(msg->joint_names.begin(), name));
    const double velocity = point.velocities[source_index];
    if (!std::isfinite(velocity)) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Velocity trajectory contains NaN or infinity");
      return;
    }
    command.velocities[index] = std::clamp(
      velocity, -max_velocity_[index], max_velocity_[index]);
  }

  command.receipt_time_ns = get_node()->now().nanoseconds();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

controller_interface::return_type RealtimeServoVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Avoid logging or allocation in this real-time callback. The controller manager
  // is configured at the FCI's fixed 1 kHz rate in both hardware YAML files.
  (void)period;

  const VelocityCommand * command = command_buffer_.readFromRT();
  std::array<double, kJointCount> target_velocity{};
  if (command->valid) {
    const double age = static_cast<double>(time.nanoseconds() - command->receipt_time_ns) * 1e-9;
    if (age >= 0.0 && age <= command_timeout_) {
      target_velocity = command->velocities;
    }
  }

  const auto previous_velocity = applied_velocity_;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    applied_velocity_[index] = limit_velocity(
      max_velocity_[index], -max_velocity_[index], max_acceleration_[index], max_jerk_[index],
      target_velocity[index], previous_velocity[index], applied_acceleration_[index]);
    // The hardware and libfranka both run at exactly 1 kHz. Track the acceleration
    // associated with the command actually sent, as required by the next limiter step.
    applied_acceleration_[index] =
      (applied_velocity_[index] - previous_velocity[index]) / kControlPeriod;
    command_interfaces_[index].set_value(applied_velocity_[index]);
  }

  if ((time - last_state_publish_time_) >= state_publish_period_) {
    publish_state(time);
    last_state_publish_time_ = time;
  }
  return controller_interface::return_type::OK;
}

bool RealtimeServoVelocityController::configure_state_message()
{
  if (!state_publisher_) {
    return false;
  }

  auto & state = state_publisher_->msg_;
  state.joint_names = joints_;
  state.reference.positions.resize(kJointCount);
  state.reference.velocities.resize(kJointCount);
  state.feedback.positions.resize(kJointCount);
  state.feedback.velocities.resize(kJointCount);
  state.error.positions.resize(kJointCount);
  state.error.velocities.resize(kJointCount);
  state.output.positions.resize(kJointCount);
  state.output.velocities.resize(kJointCount);
  state.desired.positions.resize(kJointCount);
  state.desired.velocities.resize(kJointCount);
  state.actual.positions.resize(kJointCount);
  state.actual.velocities.resize(kJointCount);
  return true;
}

void RealtimeServoVelocityController::publish_state(const rclcpp::Time & time)
{
  if (!state_publisher_ || !state_publisher_->trylock()) {
    return;
  }

  auto & state = state_publisher_->msg_;
  state.header.stamp = time;
  const VelocityCommand * command = command_buffer_.readFromRT();
  const double command_age = command->valid ?
    static_cast<double>(time.nanoseconds() - command->receipt_time_ns) * 1e-9 :
    std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const double position = state_interfaces_[2 * index].get_value();
    const double velocity = state_interfaces_[2 * index + 1].get_value();
    const double target = command_age >= 0.0 && command_age <= command_timeout_ ?
      command->velocities[index] : 0.0;

    state.reference.positions[index] = position;
    state.reference.velocities[index] = target;
    state.feedback.positions[index] = position;
    state.feedback.velocities[index] = velocity;
    state.error.positions[index] = 0.0;
    state.error.velocities[index] = target - velocity;
    state.output.positions[index] = position;
    state.output.velocities[index] = applied_velocity_[index];

    // Populate the deprecated aliases as well: existing diagnostics and bags may
    // still inspect these fields from JointTrajectoryControllerState.
    state.desired.positions[index] = position;
    state.desired.velocities[index] = applied_velocity_[index];
    state.actual.positions[index] = position;
    state.actual.velocities[index] = velocity;
  }
  state_publisher_->unlockAndPublish();
}

}  // namespace phantom_panda_teleop

PLUGINLIB_EXPORT_CLASS(
  phantom_panda_teleop::RealtimeServoVelocityController,
  controller_interface::ControllerInterface)
