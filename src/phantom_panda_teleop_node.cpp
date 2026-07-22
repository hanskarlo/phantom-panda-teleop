#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "phantom_panda_teleop/rcm_geometry.hpp"

// Butterworth low-pass filter implementation for tremor suppression
class ButterworthFilter
{
public:
  ButterworthFilter()
  : b0(0), b1(0), b2(0), a1(0), a2(0), x1(0), x2(0), y1(0), y2(0), initialized(false) {}

  void setup(double cutoff_freq, double sample_rate)
  {
    double K = std::tan(M_PI * cutoff_freq / sample_rate);
    double D = K * K + std::sqrt(2.0) * K + 1.0;
    b0 = K * K / D;
    b1 = 2.0 * K * K / D;
    b2 = K * K / D;
    a1 = 2.0 * (K * K - 1.0) / D;
    a2 = (K * K - std::sqrt(2.0) * K + 1.0) / D;
    reset();
  }

  void reset()
  {
    x1 = x2 = 0.0;
    y1 = y2 = 0.0;
    initialized = false;
  }

  double filter(double input)
  {
    if (!initialized) {
      x1 = x2 = input;
      y1 = y2 = input;
      initialized = true;
      return input;
    }
    double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    return output;
  }

private:
  double b0, b1, b2, a1, a2;
  double x1, x2, y1, y2;
  bool initialized;
};

// 3D vector low-pass filter
class Vector3ButterworthFilter
{
public:
  void setup(double cutoff_freq, double sample_rate)
  {
    filters[0].setup(cutoff_freq, sample_rate);
    filters[1].setup(cutoff_freq, sample_rate);
    filters[2].setup(cutoff_freq, sample_rate);
  }
  void reset()
  {
    filters[0].reset();
    filters[1].reset();
    filters[2].reset();
  }
  Eigen::Vector3d filter(const Eigen::Vector3d & input)
  {
    return Eigen::Vector3d(
      filters[0].filter(input.x()),
      filters[1].filter(input.y()),
      filters[2].filter(input.z())
    );
  }

private:
  ButterworthFilter filters[3];
};

enum class TeleopState
{
  HOMING = 0,
  REGISTRATION = 1,
  CLUTCHED = 2,
  ACTIVE = 3,
  SAFETY_HALT = 4
};

class PhantomPandaTeleopNode : public rclcpp::Node
{
public:
  PhantomPandaTeleopNode()
  : Node("phantom_panda_teleop_node")
  {
    // Declare and retrieve parameters
    this->declare_parameter("update_rate", 200.0);
    this->declare_parameter("haptic_base_frame", "touch_x_base");
    this->declare_parameter("haptic_ee_frame", "touch_x_ee");
    this->declare_parameter("robot_base_frame", "fr3_link0");
    this->declare_parameter("robot_ee_frame", "fr3_link8");
    this->declare_parameter("robot_tool_tip_frame", "biorob_tool_tip");
    this->declare_parameter("tip_position_scale", 0.2); // haptic-to-tip Cartesian scale
    this->declare_parameter("max_tilt_angle", 0.52); // maximum shaft tilt from base +Z
    this->declare_parameter("tip_direction_transition_depth", 0.02);
    this->declare_parameter("deadband_position", 0.0005); // 0.5 mm deadband
    this->declare_parameter("cutoff_freq", 5.0); // 5 Hz tremor low-pass cutoff
    this->declare_parameter("rcm_x", 0.4);
    this->declare_parameter("rcm_y", 0.0);
    this->declare_parameter("rcm_z", 0.2);
    this->declare_parameter("tool_length", 0.25); // 25 cm tool length
    this->declare_parameter("min_insertion_depth", 0.0);
    this->declare_parameter("max_insertion_depth", 0.23);
    this->declare_parameter("registration_insertion_depth", 0.08);
    this->declare_parameter("rcm_hole_diameter", 0.03);
    this->declare_parameter("shaft_marker_extension", 0.15);
    this->declare_parameter("rcm_warning_error", 0.002);
    this->declare_parameter("custom_tool_state_topic", "~/custom_tool_closed");
    this->declare_parameter("use_rcm", true);
    this->declare_parameter("k_linear", 0.2);
    this->declare_parameter("haptic_timeout", 0.05); // 50 ms timeout
    this->declare_parameter("robot_state_timeout", 0.10);
    this->declare_parameter("command_chain_timeout", 0.25);
    this->declare_parameter("joy_topic", "/geomagic_touch_x/joy");
    this->declare_parameter("joint_state_topic", "/franka/joint_states");
    this->declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds");
    this->declare_parameter("servo_status_topic", "/servo_node/status");
    this->declare_parameter("servo_start_service", "/servo_node/start_servo");
    this->declare_parameter(
      "servo_joint_command_topic", "/servo_node/joint_trajectory_raw");
    this->declare_parameter(
      "controller_state_topic", "/fr3_arm_controller/controller_state");
    this->declare_parameter("positioning_mode", "fixed");
    this->declare_parameter("start_in_registration", false);
    this->declare_parameter("auto_register_rcm", false);
    this->declare_parameter(
      "controller_switch_service", "/controller_manager/switch_controller");
    this->declare_parameter("arm_controller_name", "fr3_arm_controller");
    this->declare_parameter("positioning_max_linear_vel", 0.01);
    this->declare_parameter("positioning_max_angular_vel", 0.05);
    this->declare_parameter("registration_joint_velocity_tolerance", 0.01);
    this->declare_parameter("registration_settle_time", 0.5);
    this->declare_parameter<std::vector<double>>(
      "joint_velocity_halt_limits", {0.4, 0.4, 0.4, 0.4, 0.8, 0.8, 0.8});

    // Closed-loop proportional tracking gains
    this->declare_parameter("kp_pos", 12.0);
    this->declare_parameter("kp_rot", 8.0);
    this->declare_parameter("max_linear_vel", 0.15); // max Cartesian linear speed 15 cm/s
    this->declare_parameter("max_angular_vel", 0.5); // max Cartesian angular speed 0.5 rad/s
    this->declare_parameter("max_linear_accel", 0.15); // Cartesian acceleration limit [m/s^2]
    this->declare_parameter("max_angular_accel", 0.6); // Cartesian acceleration limit [rad/s^2]
    this->declare_parameter("max_linear_jerk", 1.0); // Cartesian jerk limit [m/s^3]
    this->declare_parameter("max_angular_jerk", 4.0); // Cartesian jerk limit [rad/s^3]

    this->get_parameter("update_rate", update_rate_);
    this->get_parameter("haptic_base_frame", haptic_base_frame_);
    this->get_parameter("haptic_ee_frame", haptic_ee_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("robot_ee_frame", robot_ee_frame_);
    this->get_parameter("robot_tool_tip_frame", robot_tool_tip_frame_);
    this->get_parameter("tip_position_scale", tip_position_scale_);
    this->get_parameter("max_tilt_angle", max_tilt_angle_);
    this->get_parameter("tip_direction_transition_depth", tip_direction_transition_depth_);
    this->get_parameter("deadband_position", deadband_position_);
    this->get_parameter("cutoff_freq", cutoff_freq_);
    this->get_parameter("tool_length", tool_length_);
    this->get_parameter("min_insertion_depth", min_insertion_depth_);
    this->get_parameter("max_insertion_depth", max_insertion_depth_);
    this->get_parameter("registration_insertion_depth", registration_insertion_depth_);
    this->get_parameter("rcm_hole_diameter", rcm_hole_diameter_);
    this->get_parameter("shaft_marker_extension", shaft_marker_extension_);
    this->get_parameter("rcm_warning_error", rcm_warning_error_);
    this->get_parameter("custom_tool_state_topic", custom_tool_state_topic_);
    this->get_parameter("use_rcm", use_rcm_);
    this->get_parameter("k_linear", k_linear_);
    this->get_parameter("haptic_timeout", haptic_timeout_);
    this->get_parameter("robot_state_timeout", robot_state_timeout_);
    this->get_parameter("command_chain_timeout", command_chain_timeout_);
    this->get_parameter("joy_topic", joy_topic_);
    this->get_parameter("joint_state_topic", joint_state_topic_);
    this->get_parameter("servo_twist_topic", servo_twist_topic_);
    this->get_parameter("servo_status_topic", servo_status_topic_);
    this->get_parameter("servo_start_service", servo_start_service_);
    this->get_parameter("servo_joint_command_topic", servo_joint_command_topic_);
    this->get_parameter("controller_state_topic", controller_state_topic_);
    this->get_parameter("positioning_mode", positioning_mode_);
    this->get_parameter("start_in_registration", start_in_registration_);
    this->get_parameter("auto_register_rcm", auto_register_rcm_);
    this->get_parameter("controller_switch_service", controller_switch_service_);
    this->get_parameter("arm_controller_name", arm_controller_name_);
    this->get_parameter("positioning_max_linear_vel", positioning_max_linear_vel_);
    this->get_parameter("positioning_max_angular_vel", positioning_max_angular_vel_);
    this->get_parameter(
      "registration_joint_velocity_tolerance", registration_joint_velocity_tolerance_);
    this->get_parameter("registration_settle_time", registration_settle_time_);
    this->get_parameter("joint_velocity_halt_limits", joint_velocity_halt_limits_);
    this->get_parameter("kp_pos", kp_pos_);
    this->get_parameter("kp_rot", kp_rot_);
    this->get_parameter("max_linear_vel", max_linear_vel_);
    this->get_parameter("max_angular_vel", max_angular_vel_);
    this->get_parameter("max_linear_accel", max_linear_accel_);
    this->get_parameter("max_angular_accel", max_angular_accel_);
    this->get_parameter("max_linear_jerk", max_linear_jerk_);
    this->get_parameter("max_angular_jerk", max_angular_jerk_);

    if (update_rate_ <= 0.0 || max_linear_vel_ <= 0.0 || max_angular_vel_ <= 0.0 ||
      max_linear_accel_ <= 0.0 || max_angular_accel_ <= 0.0 ||
      max_linear_jerk_ <= 0.0 || max_angular_jerk_ <= 0.0)
    {
      throw std::invalid_argument("Update rate and Cartesian motion limits must all be positive.");
    }
    if (tip_position_scale_ < 0.0 || k_linear_ < 0.0 ||
      max_tilt_angle_ < 0.0 || max_tilt_angle_ >= M_PI_2 ||
      tip_direction_transition_depth_ <= 0.0 ||
      haptic_timeout_ <= 0.0 || robot_state_timeout_ <= 0.0 || command_chain_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "Position scales must be nonnegative, max_tilt_angle must be in [0, pi/2), "
              "and transition depth/timeouts must be positive.");
    }
    if (joint_velocity_halt_limits_.size() != 7 ||
      std::any_of(
        joint_velocity_halt_limits_.begin(), joint_velocity_halt_limits_.end(),
        [](double limit) {return !std::isfinite(limit) || limit <= 0.0;}))
    {
      throw std::invalid_argument("joint_velocity_halt_limits must contain seven positive values.");
    }
    if (tool_length_ <= 0.0 || min_insertion_depth_ < 0.0 ||
      max_insertion_depth_ <= min_insertion_depth_ || max_insertion_depth_ >= tool_length_)
    {
      throw std::invalid_argument(
              "Insertion limits must satisfy 0 <= min_insertion_depth < max_insertion_depth < tool_length.");
    }
    if (registration_insertion_depth_ < min_insertion_depth_ ||
      registration_insertion_depth_ > max_insertion_depth_)
    {
      throw std::invalid_argument(
              "registration_insertion_depth must lie within the configured insertion limits.");
    }
    if (rcm_hole_diameter_ <= 0.0 || shaft_marker_extension_ < 0.0 ||
      rcm_warning_error_ < 0.0)
    {
      throw std::invalid_argument("RCM visualization dimensions must be nonnegative.");
    }
    if (positioning_mode_ != "fixed" && positioning_mode_ != "haptic_jog" &&
      positioning_mode_ != "physical_guiding")
    {
      throw std::invalid_argument(
              "positioning_mode must be fixed, haptic_jog, or physical_guiding.");
    }
    if (positioning_max_linear_vel_ <= 0.0 || positioning_max_angular_vel_ <= 0.0 ||
      registration_joint_velocity_tolerance_ < 0.0 || registration_settle_time_ < 0.0)
    {
      throw std::invalid_argument(
              "Positioning limits must be positive and registration guards nonnegative.");
    }

    this->declare_parameter("maximize_collision_thresholds", false);
    this->get_parameter("maximize_collision_thresholds", maximize_collision_thresholds_);

    set_collision_behavior_client_ =
      this->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "/service_server/set_full_collision_behavior");

    // Setup output twist command filtering
    twist_linear_filter_.setup(cutoff_freq_, update_rate_);
    twist_angular_filter_.setup(cutoff_freq_, update_rate_);

    // Set initial RCM coordinates
    rcm_position_ = Eigen::Vector3d(
      this->get_parameter("rcm_x").as_double(),
      this->get_parameter("rcm_y").as_double(),
      this->get_parameter("rcm_z").as_double()
    );

    // Initialize haptic-to-robot frame rotation matrix
    R_h_r_ << 0.0, 0.0, -1.0,
      -1.0, 0.0, 0.0,
      0.0, 1.0, 0.0;

    // Setup filtering
    butterworth_filter_.setup(cutoff_freq_, update_rate_);

    // Setup TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup Subscribers & Publishers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PhantomPandaTeleopNode::joy_callback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PhantomPandaTeleopNode::joint_state_callback, this, std::placeholders::_1));

    // MoveIt Servo Cartesian command input publisher
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      servo_twist_topic_, 10);

    // Servo reports singularity, collision, and joint-bound conditions on this topic.
    servo_status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      servo_status_topic_, 10,
      std::bind(&PhantomPandaTeleopNode::servo_status_callback, this, std::placeholders::_1));

    // Observe both sides of the Servo/controller boundary. These subscriptions do not alter
    // commands; they make a disconnected or non-executing real-hardware pipeline explicit.
    servo_joint_command_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      servo_joint_command_topic_, 10,
      std::bind(
        &PhantomPandaTeleopNode::servo_joint_command_callback, this,
        std::placeholders::_1));
    controller_state_sub_ =
      this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      controller_state_topic_, 10,
      std::bind(&PhantomPandaTeleopNode::controller_state_callback, this, std::placeholders::_1));

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    teleop_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("~/state", state_qos);
    custom_tool_state_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(custom_tool_state_topic_, state_qos);
    rcm_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/rcm_markers", 10);
    rcm_error_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/rcm_error", 10);

    // Setup calibration services
    register_rcm_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/register_rcm",
      std::bind(
        &PhantomPandaTeleopNode::register_rcm_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    proceed_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/proceed",
      std::bind(
        &PhantomPandaTeleopNode::proceed_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Initialize state
    state_ = use_rcm_ && start_in_registration_ ?
      TeleopState::REGISTRATION : TeleopState::HOMING;
    homing_timer_ = 0.0;
    custom_tool_closed_ = false;
    prev_joy_button1_ = 0;
    prev_joy_button2_ = 0;
    servo_started_ = false;
    servo_start_request_pending_ = false;
    command_limiter_initialized_ = false;
    have_recent_joint_state_ = false;
    have_servo_joint_command_ = false;
    have_controller_state_ = false;
    rcm_registered_ = false;
    haptic_positioning_active_ = false;
    button1_pressed_ = false;
    controller_switch_pending_ = false;
    guiding_controller_released_ = positioning_mode_ != "physical_guiding";
    latest_joint_velocities_.fill(0.0);
    last_joint_motion_time_ = this->get_clock()->now();
    last_controller_switch_attempt_ = this->get_clock()->now();
    last_auto_registration_attempt_ = this->get_clock()->now();
    max_controller_position_error_ = 0.0;
    max_controller_velocity_error_ = 0.0;
    publish_custom_tool_state();
    start_servo_client_ = this->create_client<std_srvs::srv::Trigger>(servo_start_service_);
    controller_switch_client_ =
      this->create_client<controller_manager_msgs::srv::SwitchController>(
      controller_switch_service_);

    // Run core control loop
    double loop_period_ms = 1000.0 / update_rate_;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(loop_period_ms),
      std::bind(&PhantomPandaTeleopNode::control_loop, this));
    marker_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PhantomPandaTeleopNode::publish_rcm_visualization, this));

    RCLCPP_INFO(
      this->get_logger(), "BioRob teleop node initialized in %s mode. RCM Point: [%.3f, %.3f, %.3f]",
      state_ == TeleopState::REGISTRATION ? "REGISTRATION" : "HOMING",
      rcm_position_.x(), rcm_position_.y(), rcm_position_.z());
    RCLCPP_INFO(
      this->get_logger(),
      "Command chain: %s -> %s -> %s",
      servo_twist_topic_.c_str(), servo_joint_command_topic_.c_str(),
      controller_state_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Registration: positioning=%s, insertion marker=%.1f mm, automatic=%s",
      positioning_mode_.c_str(), registration_insertion_depth_ * 1000.0,
      auto_register_rcm_ ? "true" : "false");
  }

private:
  bool robot_is_stationary(std::string & reason)
  {
    if (!have_recent_joint_state_) {
      reason = "No complete arm joint-state feedback has been received.";
      return false;
    }

    const rclcpp::Time now = this->get_clock()->now();
    const double feedback_age = (now - last_joint_state_receipt_time_).seconds();
    if (feedback_age < 0.0 || feedback_age > robot_state_timeout_) {
      reason = "Arm joint-state feedback is stale.";
      return false;
    }

    double max_velocity = 0.0;
    for (const double velocity : latest_joint_velocities_) {
      if (!std::isfinite(velocity)) {
        reason = "Arm joint-state feedback contains a non-finite velocity.";
        return false;
      }
      max_velocity = std::max(max_velocity, std::abs(velocity));
    }
    if (max_velocity > registration_joint_velocity_tolerance_) {
      std::ostringstream stream;
      stream << "Robot is still moving (maximum joint speed " << std::fixed <<
        std::setprecision(4) << max_velocity << " rad/s).";
      reason = stream.str();
      return false;
    }

    const double stationary_time = (now - last_joint_motion_time_).seconds();
    if (stationary_time < registration_settle_time_) {
      std::ostringstream stream;
      stream << "Robot must remain stationary for " << std::fixed << std::setprecision(2) <<
        registration_settle_time_ << " s before registration.";
      reason = stream.str();
      return false;
    }
    return true;
  }

  bool request_arm_controller_switch(bool activate)
  {
    if (controller_switch_pending_) {
      return false;
    }
    if (!controller_switch_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for controller switch service %s.", controller_switch_service_.c_str());
      return false;
    }

    auto request =
      std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    if (activate) {
      request->activate_controllers = {arm_controller_name_};
    } else {
      request->deactivate_controllers = {arm_controller_name_};
    }
    request->strictness =
      controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 5;
    request->timeout.nanosec = 0;

    controller_switch_pending_ = true;
    last_controller_switch_attempt_ = this->get_clock()->now();
    controller_switch_client_->async_send_request(
      request,
      [this, activate](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        controller_switch_pending_ = false;
        try {
          const auto response = future.get();
          if (!response->ok) {
            RCLCPP_ERROR(
              this->get_logger(), "Failed to %s arm controller %s.",
              activate ? "activate" : "deactivate", arm_controller_name_.c_str());
            return;
          }

          if (activate) {
            guiding_controller_released_ = false;
            have_controller_state_ = false;
            RCLCPP_INFO(
              this->get_logger(), "Arm controller %s reactivated; robot is held.",
              arm_controller_name_.c_str());
            if (state_ == TeleopState::REGISTRATION && (rcm_registered_ || !use_rcm_)) {
              state_ = TeleopState::CLUTCHED;
              RCLCPP_INFO(
                this->get_logger(),
                "Transitioned from REGISTRATION to CLUTCHED state.");
            }
          } else {
            guiding_controller_released_ = true;
            have_controller_state_ = false;
            RCLCPP_WARN(
              this->get_logger(),
              "Arm command controller is inactive. Use the Franka guiding buttons to "
              "position the tool; do not proceed until the arm is stationary.");
          }
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            this->get_logger(), "Controller switch request failed: %s", exception.what());
        }
      });
    return true;
  }

  // Service to dynamically register Trocar RCM point
  void register_rcm_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!use_rcm_) {
      response->success = false;
      response->message = "RCM constraint is currently disabled.";
      return;
    }
    if (state_ != TeleopState::REGISTRATION) {
      response->success = false;
      response->message = "RCM registration is only allowed in REGISTRATION state.";
      return;
    }
    if (button1_pressed_) {
      response->success = false;
      response->message = "Release the haptic clutch before registering the RCM.";
      return;
    }
    std::string stationary_reason;
    if (!robot_is_stationary(stationary_reason)) {
      response->success = false;
      response->message = stationary_reason;
      return;
    }
    if (positioning_mode_ == "physical_guiding" && !guiding_controller_released_) {
      response->success = false;
      response->message =
        "The arm controller must remain inactive until the physical guiding pose is captured.";
      return;
    }
    if (positioning_mode_ == "physical_guiding" &&
      !controller_switch_client_->service_is_ready())
    {
      response->success = false;
      response->message = "Controller switch service is unavailable; refusing to capture.";
      return;
    }

    // Capture the explicit tool-tip frame after the known shaft marker has been placed
    // at the trocar. The tip is already registration_insertion_depth_ beyond the port.
    geometry_msgs::msg::TransformStamped robot_tf;
    geometry_msgs::msg::TransformStamped tool_tip_tf;
    try {
      robot_tf =
        tf_buffer_->lookupTransform(robot_base_frame_, robot_ee_frame_, tf2::TimePointZero);
      tool_tip_tf =
        tf_buffer_->lookupTransform(robot_base_frame_, robot_tool_tip_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      response->success = false;
      response->message = "Failed to lookup flange/tool-tip transform: " + std::string(ex.what());
      return;
    }

    Eigen::Vector3d p_ee(
      robot_tf.transform.translation.x,
      robot_tf.transform.translation.y,
      robot_tf.transform.translation.z
    );

    Eigen::Quaterniond q_ee(
      robot_tf.transform.rotation.w,
      robot_tf.transform.rotation.x,
      robot_tf.transform.rotation.y,
      robot_tf.transform.rotation.z
    );
    if (!q_ee.coeffs().allFinite() || q_ee.norm() < 1e-9) {
      response->success = false;
      response->message = "Flange orientation is invalid.";
      return;
    }
    q_ee.normalize();

    const Eigen::Vector3d p_tip(
      tool_tip_tf.transform.translation.x,
      tool_tip_tf.transform.translation.y,
      tool_tip_tf.transform.translation.z);
    const Eigen::Vector3d flange_to_tip = p_tip - p_ee;
    const double measured_tool_length = flange_to_tip.norm();
    const Eigen::Vector3d z_ee = q_ee.toRotationMatrix().col(2).normalized();
    if (!std::isfinite(measured_tool_length) || measured_tool_length <= max_insertion_depth_) {
      response->success = false;
      response->message = "Tool-tip frame produces an invalid tool length.";
      return;
    }

    const double axis_alignment = flange_to_tip.normalized().dot(z_ee);
    if (axis_alignment < std::cos(1.0 * M_PI / 180.0)) {
      response->success = false;
      response->message = "Tool-tip frame is not aligned with the flange local +Z axis.";
      return;
    }

    if (std::abs(measured_tool_length - tool_length_) > 0.001) {
      RCLCPP_WARN(
        this->get_logger(),
        "Configured tool length %.4f m differs from TF length %.4f m; using the TF length.",
        tool_length_, measured_tool_length);
      tool_length_ = measured_tool_length;
      this->set_parameter(rclcpp::Parameter("tool_length", tool_length_));
    }

    rcm_position_ = phantom_panda_teleop::rcmFromInsertedToolTip(
      p_tip, z_ee, registration_insertion_depth_);
    rcm_registered_ = true;
    this->set_parameters(
    {
      rclcpp::Parameter("rcm_x", rcm_position_.x()),
      rclcpp::Parameter("rcm_y", rcm_position_.y()),
      rclcpp::Parameter("rcm_z", rcm_position_.z())});

    RCLCPP_INFO(
      this->get_logger(),
      "Registered RCM trocar center at [%.3f, %.3f, %.3f] from %.1f mm insertion marker.",
      rcm_position_.x(), rcm_position_.y(), rcm_position_.z(),
      registration_insertion_depth_ * 1000.0);

    if (positioning_mode_ == "physical_guiding") {
      if (!request_arm_controller_switch(true)) {
        response->success = false;
        response->message =
          "RCM captured, but the arm-controller reactivation request could not be sent.";
        return;
      }
      response->success = true;
      response->message =
        "RCM captured. Reactivating the arm controller; wait for CLUTCHED state.";
    } else {
      state_ = TeleopState::CLUTCHED;
      response->success = true;
      response->message = "Trocar / RCM point registered successfully.";
      RCLCPP_INFO(this->get_logger(), "Transitioned from REGISTRATION to CLUTCHED state.");
    }
  }

  void register_rcm_from_haptic_button()
  {
    const bool entered_from_homing = state_ == TeleopState::HOMING;
    if (!entered_from_homing && state_ != TeleopState::REGISTRATION) {
      RCLCPP_WARN(
        this->get_logger(),
        "Button 2 RCM registration is only available during HOMING or REGISTRATION.");
      return;
    }

    // Use the same guarded capture path as the service. A failed direct capture returns
    // to HOMING so the operator can resume jogging and correct the alignment.
    if (entered_from_homing) {
      state_ = TeleopState::REGISTRATION;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    register_rcm_callback(request, response);
    if (!response->success && entered_from_homing) {
      state_ = TeleopState::HOMING;
    }

    if (response->success) {
      RCLCPP_INFO(
        this->get_logger(), "Button 2 registration: %s", response->message.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Button 2 registration rejected: %s", response->message.c_str());
    }
  }

  void try_auto_register_rcm()
  {
    const rclcpp::Time now = this->get_clock()->now();
    if ((now - last_auto_registration_attempt_).seconds() < 0.5) {
      return;
    }
    last_auto_registration_attempt_ = now;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    register_rcm_callback(request, response);
    if (response->success) {
      RCLCPP_INFO(
        this->get_logger(), "Automatic inserted-tool registration: %s",
        response->message.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting to auto-register the inserted tool: %s", response->message.c_str());
    }
  }

  void proceed_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (state_ == TeleopState::HOMING) {
      if (button1_pressed_ || haptic_positioning_active_) {
        response->success = false;
        response->message =
          "Release the haptic positioning clutch before leaving HOMING.";
        return;
      }
      std::string stationary_reason;
      if (!robot_is_stationary(stationary_reason)) {
        response->success = false;
        response->message = stationary_reason;
        return;
      }
      if (positioning_mode_ == "physical_guiding" && !guiding_controller_released_) {
        response->success = false;
        response->message =
          "Waiting for the arm controller to deactivate before physical guiding.";
        return;
      }
      if (use_rcm_) {
        state_ = TeleopState::REGISTRATION;
        response->success = true;
        response->message =
          "Transitioned from HOMING to REGISTRATION. The pose is locked for RCM capture.";
      } else if (positioning_mode_ == "physical_guiding") {
        state_ = TeleopState::REGISTRATION;
        if (!request_arm_controller_switch(true)) {
          state_ = TeleopState::HOMING;
          response->success = false;
          response->message = "Could not request arm-controller reactivation.";
          return;
        }
        response->success = true;
        response->message = "Reactivating the arm controller; wait for CLUTCHED state.";
      } else {
        state_ = TeleopState::CLUTCHED;
        response->success = true;
        response->message = "Transitioned from HOMING to CLUTCHED.";
      }
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else if (state_ == TeleopState::SAFETY_HALT) {
      state_ = TeleopState::HOMING;
      haptic_positioning_active_ = false;
      if (positioning_mode_ == "physical_guiding") {
        guiding_controller_released_ = false;
      }
      response->success = true;
      response->message =
        "Reset from SAFETY_HALT to HOMING. Please ensure haptic device is connected and healthy.";
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
      response->success = false;
      response->message = "Proceed command is only valid in HOMING or SAFETY_HALT state.";
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons.size() < 2) {return;}

    int button1 = msg->buttons[0]; // Front button (Clutch deadman switch)
    int button2 = msg->buttons[1]; // Rear button (registration/custom-tool toggle)
    button1_pressed_ = button1 == 1;

    // Check for haptic device warnings (e.g. max velocity exceeded)
    if (msg->buttons.size() >= 3 && msg->buttons[2] == 1) {
      if (state_ == TeleopState::ACTIVE) {
        RCLCPP_WARN(
          this->get_logger(),
          "Haptic device exceeded max velocity! Disengaging active control and transitioning to CLUTCHED.");
        state_ = TeleopState::CLUTCHED;
      }
      if (haptic_positioning_active_) {
        haptic_positioning_active_ = false;
        publish_zero_velocity();
        RCLCPP_WARN(
          this->get_logger(),
          "Haptic velocity warning stopped HOMING positioning motion.");
      }
      prev_joy_button1_ = button1;
      prev_joy_button2_ = button2;
      return;
    }

    // Button 1 is a positioning deadman before RCM registration. The same deliberately
    // slow, unconstrained jog is used with either the simulated or physical haptic device.
    if (state_ == TeleopState::HOMING && positioning_mode_ == "haptic_jog") {
      if (button1 == 1 && prev_joy_button1_ == 0) {
        if (initialize_active_mapping(false)) {
          haptic_positioning_active_ = true;
          RCLCPP_INFO(
            this->get_logger(),
            "HOMING positioning clutch engaged; RCM constraint is not yet active.");
        }
      } else if (button1 == 0 && haptic_positioning_active_) {
        haptic_positioning_active_ = false;
        publish_zero_velocity();
        RCLCPP_INFO(
          this->get_logger(),
          "HOMING positioning clutch released; robot held for registration.");
      }
    }

    // Handle deadman clutch logic
    if (state_ == TeleopState::CLUTCHED && button1 == 1 && prev_joy_button1_ == 0) {
      // Transition to ACTIVE on rising edge of button1
      if (initialize_active_mapping()) {
        state_ = TeleopState::ACTIVE;
        RCLCPP_INFO(this->get_logger(), "Clutch engaged -> ACTIVE state.");
      }
    } else if (state_ == TeleopState::ACTIVE && button1 == 0) {
      // Transition back to CLUTCHED
      state_ = TeleopState::CLUTCHED;
      RCLCPP_INFO(this->get_logger(), "Clutch disengaged -> CLUTCHED state. Freezing robot.");
    }

    // Button 2 registers the RCM during positioning. Once registration succeeds, its
    // normal role is restored as the placeholder custom-tool open/close command.
    if (button2 == 1 && prev_joy_button2_ == 0) {
      if (state_ == TeleopState::HOMING || state_ == TeleopState::REGISTRATION) {
        register_rcm_from_haptic_button();
      } else if (state_ == TeleopState::CLUTCHED || state_ == TeleopState::ACTIVE) {
        toggle_custom_tool();
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Button 2 ignored while the teleoperation system is halted.");
      }
    }
    prev_joy_button1_ = button1;
    prev_joy_button2_ = button2;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.size() != msg->velocity.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring arm joint state with missing velocity data.");
      return;
    }

    std::array<double, 7> measured_velocities{};
    for (size_t joint_index = 0; joint_index < joint_velocity_halt_limits_.size(); ++joint_index) {
      const std::string joint_name = "fr3_joint" + std::to_string(joint_index + 1);
      const auto name_it = std::find(msg->name.begin(), msg->name.end(), joint_name);
      if (name_it == msg->name.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Ignoring incomplete /franka/joint_states feedback.");
        return;
      }

      const size_t state_index = static_cast<size_t>(std::distance(msg->name.begin(), name_it));
      measured_velocities[joint_index] = msg->velocity[state_index];
    }

    const rclcpp::Time joint_state_time = this->get_clock()->now();
    latest_joint_velocities_ = measured_velocities;
    const double maximum_measured_velocity = *std::max_element(
      measured_velocities.begin(), measured_velocities.end(),
      [](double left, double right) {return std::abs(left) < std::abs(right);});
    if (!std::isfinite(maximum_measured_velocity) ||
      std::abs(maximum_measured_velocity) > registration_joint_velocity_tolerance_)
    {
      last_joint_motion_time_ = joint_state_time;
    }
    last_joint_state_receipt_time_ = joint_state_time;
    have_recent_joint_state_ = true;
    if (state_ != TeleopState::ACTIVE && !haptic_positioning_active_) {
      return;
    }

    for (size_t joint_index = 0; joint_index < measured_velocities.size(); ++joint_index) {
      const std::string joint_name = "fr3_joint" + std::to_string(joint_index + 1);
      const double velocity = measured_velocities[joint_index];
      if (!std::isfinite(velocity) ||
        std::abs(velocity) > joint_velocity_halt_limits_[joint_index])
      {
        state_ = TeleopState::SAFETY_HALT;
        haptic_positioning_active_ = false;
        publish_zero_velocity();
        RCLCPP_ERROR(
          this->get_logger(),
          "SAFETY HALT: %s measured velocity %.3f rad/s exceeded teleop guard %.3f rad/s.",
          joint_name.c_str(), velocity, joint_velocity_halt_limits_[joint_index]);
        return;
      }
    }
  }

  void servo_status_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    // moveit_servo::StatusCode values in ROS 2 Humble:
    // 2 = HALT_FOR_SINGULARITY, 4 = HALT_FOR_COLLISION, 5 = JOINT_BOUND.
    const int8_t status = msg->data;
    if (status == 1 || status == 3 || status == 6) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "MoveIt Servo is decelerating for a kinematic/collision warning (status=%d).",
        static_cast<int>(status));
      return;
    }

    if ((status == 2 || status == 4 || status == 5) &&
      (state_ == TeleopState::ACTIVE || haptic_positioning_active_))
    {
      state_ = TeleopState::SAFETY_HALT;
      haptic_positioning_active_ = false;
      reset_command_limiter();
      publish_zero_velocity();
      RCLCPP_ERROR(
        this->get_logger(),
        "SAFETY HALT: MoveIt Servo reported a hard stop (status=%d). "
        "Move the robot away from singularities, collisions, and joint bounds before resetting.",
        static_cast<int>(status));
    }
  }

  void servo_joint_command_callback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->joint_names.empty() || msg->points.empty()) {
      return;
    }
    last_servo_joint_command_time_ = this->get_clock()->now();
    have_servo_joint_command_ = true;
  }

  void controller_state_callback(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    last_controller_state_time_ = this->get_clock()->now();
    have_controller_state_ = true;
    max_controller_position_error_ = 0.0;
    max_controller_velocity_error_ = 0.0;

    for (const double error : msg->error.positions) {
      if (std::isfinite(error)) {
        max_controller_position_error_ =
          std::max(max_controller_position_error_, std::abs(error));
      }
    }
    for (const double error : msg->error.velocities) {
      if (std::isfinite(error)) {
        max_controller_velocity_error_ =
          std::max(max_controller_velocity_error_, std::abs(error));
      }
    }
  }

  bool initialize_active_mapping(bool enforce_rcm = true)
  {
    if (!servo_started_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot initialize ACTIVE mapping: MoveIt Servo start has not been confirmed.");
      return false;
    }
    if (twist_pub_->get_subscription_count() == 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot initialize ACTIVE mapping: no subscriber on Servo input %s.",
        servo_twist_topic_.c_str());
      return false;
    }
    // One subscriber is this diagnostic observer; a second must be the bounded
    // trajectory accumulator that feeds the arm controller.
    if (this->count_subscribers(servo_joint_command_topic_) <= 1) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot initialize ACTIVE mapping: arm controller is not subscribed to %s. "
        "Check that fr3_arm_controller is active and topic namespaces match.",
        servo_joint_command_topic_.c_str());
      return false;
    }

    const rclcpp::Time preflight_now = this->get_clock()->now();
    if (!have_controller_state_ ||
      (preflight_now - last_controller_state_time_).seconds() > command_chain_timeout_)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot initialize ACTIVE mapping: no recent controller state on %s. "
        "The fr3_arm_controller may be inactive or misconfigured.",
        controller_state_topic_.c_str());
      return false;
    }

    // Lookup starting transforms to register clutch offsets
    geometry_msgs::msg::TransformStamped haptic_tf;
    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      haptic_tf = tf_buffer_->lookupTransform(
        haptic_base_frame_, haptic_ee_frame_,
        tf2::TimePointZero);
      robot_tf =
        tf_buffer_->lookupTransform(robot_base_frame_, robot_ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not initialize ACTIVE mapping: %s", ex.what());
      return false;
    }

    // Check if haptic TF is stale before starting
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time haptic_time = haptic_tf.header.stamp;
    double age = (now - haptic_time).seconds();
    if (age > haptic_timeout_) {
      RCLCPP_ERROR(
        this->get_logger(), "Cannot initialize ACTIVE mapping: Haptic transform is stale (age: %.3f s, timeout: %.3f s).", age,
        haptic_timeout_);
      return false;
    }

    const double robot_age = (now - rclcpp::Time(robot_tf.header.stamp)).seconds();
    if (robot_age > robot_state_timeout_ || robot_age < -0.01) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot initialize ACTIVE mapping: Robot transform is stale or time-invalid "
        "(age: %.3f s, timeout: %.3f s).",
        robot_age, robot_state_timeout_);
      return false;
    }
    if (!have_recent_joint_state_ ||
      (now - last_joint_state_receipt_time_).seconds() > robot_state_timeout_)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot initialize ACTIVE mapping: No recent /franka/joint_states velocity feedback.");
      return false;
    }

    // Register initial haptic coordinates
    haptic_start_pos_ = Eigen::Vector3d(
      haptic_tf.transform.translation.x,
      haptic_tf.transform.translation.y,
      haptic_tf.transform.translation.z
    );

    // Register initial robot state
    robot_start_pos_ = Eigen::Vector3d(
      robot_tf.transform.translation.x,
      robot_tf.transform.translation.y,
      robot_tf.transform.translation.z
    );

    robot_start_rot_ = Eigen::Quaterniond(
      robot_tf.transform.rotation.w,
      robot_tf.transform.rotation.x,
      robot_tf.transform.rotation.y,
      robot_tf.transform.rotation.z
    );
    robot_start_rot_.normalize();

    if (enforce_rcm && use_rcm_) {
      if (!rcm_registered_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Cannot initialize ACTIVE mapping before the tool-tip RCM registration succeeds.");
        return false;
      }

      const auto start_state = phantom_panda_teleop::sphericalStateFromFlange(
        robot_start_pos_, rcm_position_, tool_length_);
      double start_insertion = start_state.insertion;
      if (std::abs(start_insertion) < 1e-6) {
        start_insertion = 0.0; // remove registration/TF roundoff at the tool-tip boundary
      }
      const Eigen::Vector3d u_start = (robot_start_pos_ - rcm_position_).normalized();

      if (start_insertion < min_insertion_depth_ || start_insertion > max_insertion_depth_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Initial insertion %.3f m is outside configured limits [%.3f, %.3f] m.",
          start_insertion, min_insertion_depth_, max_insertion_depth_);
        return false;
      }
      if (start_state.phi > max_tilt_angle_) {
        RCLCPP_ERROR(
          this->get_logger(), "Robot is tilted too far from RCM constraint (phi = %.2f rad). Please reposition.",
          start_state.phi);
        return false;
      }

      // The haptic controls only the Cartesian tool-tip position. Preserve the current
      // ideal tip as the clutch-relative origin and retain the shaft direction for the
      // zero-insertion case, where a point at the RCM cannot define an orientation.
      rcm_start_shaft_axis_ = u_start;
      robot_start_tip_pos_ = rcm_position_ - start_insertion * u_start;

      // Preserve the measured starting orientation and transport its tool axis continuously.
      // This avoids the cross-product pole singularity that occurs when the tool is vertical.
      rcm_start_z_axis_ = robot_start_rot_.toRotationMatrix().col(2).normalized();
      const Eigen::Vector3d expected_start_z = -u_start;
      const double axis_alignment = std::clamp(rcm_start_z_axis_.dot(expected_start_z), -1.0, 1.0);
      const double axis_error = std::acos(axis_alignment);
      if (axis_error > 0.0873) { // 5 degrees
        RCLCPP_ERROR(
          this->get_logger(),
          "Robot tool axis misses the registered RCM direction by %.2f deg. Re-register the RCM.",
          axis_error * 180.0 / M_PI);
        return false;
      }

      // Log tracking target info
      RCLCPP_INFO(
        this->get_logger(),
        "Cartesian tip mapping locked (RCM active): tip0=[%.3f, %.3f, %.3f], "
        "insertion0=%.3f m, scale=%.3f",
        robot_start_tip_pos_.x(), robot_start_tip_pos_.y(), robot_start_tip_pos_.z(),
        start_insertion, tip_position_scale_);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Teleop values locked (direct positioning): robot_start_pos=[%.3f, %.3f, %.3f]",
        robot_start_pos_.x(), robot_start_pos_.y(), robot_start_pos_.z());
    }

    // Filter reset
    butterworth_filter_.reset();
    twist_linear_filter_.reset();
    twist_angular_filter_.reset();
    reset_command_limiter();
    // Call service to maximize collision thresholds if configured
    if (maximize_collision_thresholds_) {
      maximize_collision_thresholds();
    }
    active_mapping_start_time_ = this->get_clock()->now();
    return true;
  }

  void publish_custom_tool_state()
  {
    std_msgs::msg::Bool state_message;
    state_message.data = custom_tool_closed_;
    custom_tool_state_pub_->publish(state_message);
  }

  void toggle_custom_tool()
  {
    custom_tool_closed_ = !custom_tool_closed_;
    publish_custom_tool_state();
    RCLCPP_INFO(
      this->get_logger(),
      "Placeholder custom tool command: %s. No physical tool actuator is connected yet.",
      custom_tool_closed_ ? "CLOSE" : "OPEN");
  }

  void maximize_collision_thresholds()
  {
    if (!set_collision_behavior_client_->service_is_ready()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Collision behavior service '/service_server/set_full_collision_behavior' not ready.");
      return;
    }

    auto request = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();
    // Maximize nominal and acceleration thresholds for both torque and forces
    std::fill(
      request->lower_torque_thresholds_acceleration.begin(),
      request->lower_torque_thresholds_acceleration.end(), 100.0);
    std::fill(
      request->upper_torque_thresholds_acceleration.begin(),
      request->upper_torque_thresholds_acceleration.end(), 100.0);
    std::fill(
      request->lower_torque_thresholds_nominal.begin(),
      request->lower_torque_thresholds_nominal.end(), 100.0);
    std::fill(
      request->upper_torque_thresholds_nominal.begin(),
      request->upper_torque_thresholds_nominal.end(), 100.0);
    std::fill(
      request->lower_force_thresholds_acceleration.begin(),
      request->lower_force_thresholds_acceleration.end(), 100.0);
    std::fill(
      request->upper_force_thresholds_acceleration.begin(),
      request->upper_force_thresholds_acceleration.end(), 100.0);
    std::fill(
      request->lower_force_thresholds_nominal.begin(),
      request->lower_force_thresholds_nominal.end(), 100.0);
    std::fill(
      request->upper_force_thresholds_nominal.begin(),
      request->upper_force_thresholds_nominal.end(), 100.0);

    set_collision_behavior_client_->async_send_request(
      request,
      [this](rclcpp::Client<franka_msgs::srv::SetFullCollisionBehavior>::SharedFuture future) {
        try {
          auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(
              this->get_logger(),
              "Successfully maximized Franka force/torque collision thresholds to 100.0.");
          } else {
            RCLCPP_ERROR(
              this->get_logger(),
              "Franka controller failed to set collision thresholds: %s", response->error.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Service call to set collision behavior threw an exception: %s", e.what());
        }
      });
  }

  void publish_rcm_visualization()
  {
    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      robot_tf =
        tf_buffer_->lookupTransform(robot_base_frame_, robot_ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }

    const Eigen::Vector3d p_ee(
      robot_tf.transform.translation.x,
      robot_tf.transform.translation.y,
      robot_tf.transform.translation.z);
    Eigen::Quaterniond q_ee(
      robot_tf.transform.rotation.w,
      robot_tf.transform.rotation.x,
      robot_tf.transform.rotation.y,
      robot_tf.transform.rotation.z);
    if (!q_ee.coeffs().allFinite() || q_ee.norm() < 1e-9) {
      return;
    }
    q_ee.normalize();
    const Eigen::Vector3d shaft_axis = q_ee.toRotationMatrix().col(2).normalized();
    const double rcm_error = phantom_panda_teleop::pointToShaftLineDistance(
      rcm_position_, p_ee, shaft_axis);
    const Eigen::Vector3d closest_point = phantom_panda_teleop::closestPointOnShaftLine(
      rcm_position_, p_ee, shaft_axis);

    std_msgs::msg::Float64 error_msg;
    error_msg.data = rcm_error;
    rcm_error_pub_->publish(error_msg);

    if (rcm_registered_ && state_ == TeleopState::ACTIVE && rcm_error > rcm_warning_error_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Measured shaft misses the registered RCM center by %.2f mm.", rcm_error * 1000.0);
    }

    const auto to_point = [](const Eigen::Vector3d & vector) {
        geometry_msgs::msg::Point point;
        point.x = vector.x();
        point.y = vector.y();
        point.z = vector.z();
        return point;
      };
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = robot_base_frame_;
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker ring;
    ring.header = header;
    ring.ns = "rcm_hole";
    ring.id = 0;
    ring.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ring.action = visualization_msgs::msg::Marker::ADD;
    ring.pose.position = to_point(rcm_position_);
    ring.pose.orientation.w = 1.0;
    ring.scale.x = 0.002;
    constexpr int ring_segments = 64;
    const double radius = rcm_hole_diameter_ / 2.0;
    for (int index = 0; index <= ring_segments; ++index) {
      const double angle = 2.0 * M_PI * static_cast<double>(index) / ring_segments;
      geometry_msgs::msg::Point point;
      point.x = radius * std::cos(angle);
      point.y = radius * std::sin(angle);
      point.z = 0.0;
      ring.points.push_back(point);
    }
    if (!rcm_registered_) {
      ring.color.r = 1.0;
      ring.color.g = 0.65;
    } else if (rcm_error <= rcm_warning_error_) {
      ring.color.g = 1.0;
      ring.color.b = 0.15;
    } else {
      ring.color.r = 1.0;
      ring.color.g = 0.1;
    }
    ring.color.a = 0.95;
    markers.markers.push_back(ring);

    visualization_msgs::msg::Marker center;
    center.header = header;
    center.ns = "rcm_center";
    center.id = 1;
    center.type = visualization_msgs::msg::Marker::SPHERE;
    center.action = visualization_msgs::msg::Marker::ADD;
    center.pose.position = to_point(rcm_position_);
    center.pose.orientation.w = 1.0;
    center.scale.x = center.scale.y = center.scale.z = 0.005;
    center.color = ring.color;
    markers.markers.push_back(center);

    visualization_msgs::msg::Marker shaft;
    shaft.header = header;
    shaft.ns = "shaft_centerline";
    shaft.id = 2;
    shaft.type = visualization_msgs::msg::Marker::LINE_STRIP;
    shaft.action = visualization_msgs::msg::Marker::ADD;
    shaft.pose.orientation.w = 1.0;
    shaft.scale.x = 0.002;
    shaft.color.r = 0.05;
    shaft.color.g = 0.75;
    shaft.color.b = 1.0;
    shaft.color.a = 0.95;
    shaft.points.push_back(to_point(p_ee - 0.03 * shaft_axis));
    shaft.points.push_back(
      to_point(p_ee + (tool_length_ + shaft_marker_extension_) * shaft_axis));
    markers.markers.push_back(shaft);

    visualization_msgs::msg::Marker residual;
    residual.header = header;
    residual.ns = "rcm_error";
    residual.id = 3;
    residual.type = visualization_msgs::msg::Marker::LINE_LIST;
    residual.action = visualization_msgs::msg::Marker::ADD;
    residual.pose.orientation.w = 1.0;
    residual.scale.x = 0.003;
    residual.color.r = 1.0;
    residual.color.g = 0.05;
    residual.color.a = 1.0;
    residual.points.push_back(to_point(rcm_position_));
    residual.points.push_back(to_point(closest_point));
    markers.markers.push_back(residual);

    visualization_msgs::msg::Marker label;
    label.header = header;
    label.ns = "rcm_label";
    label.id = 4;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position = to_point(rcm_position_ + Eigen::Vector3d(0.0, 0.0, 0.025));
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.014;
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0;
    std::ostringstream label_stream;
    label_stream << (rcm_registered_ ? "RCM" : "RCM candidate") << " | shaft miss "
                 << std::fixed << std::setprecision(2) << rcm_error * 1000.0 << " mm";
    label.text = label_stream.str();
    markers.markers.push_back(label);

    rcm_marker_pub_->publish(markers);
  }

  void start_moveit_servo()
  {
    if (servo_started_ || servo_start_request_pending_) {return;}

    if (!start_servo_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for /servo_node/start_servo service...");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    servo_start_request_pending_ = true;

    start_servo_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        servo_start_request_pending_ = false;
        try {
          auto response = future.get();
          if (response->success) {
            servo_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Successfully started MoveIt Servo.");
          } else {
            RCLCPP_ERROR(
              this->get_logger(), "Failed to start MoveIt Servo: %s",
              response->message.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "MoveIt Servo start service failed: %s", e.what());
        }
      });
  }

  void control_loop()
  {
    start_moveit_servo();

    std_msgs::msg::Int8 state_msg;
    state_msg.data = static_cast<int8_t>(state_);
    teleop_state_pub_->publish(state_msg);

    if (state_ == TeleopState::SAFETY_HALT) {
      publish_zero_velocity();
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1000,
        "SAFETY HALT: Teleoperation halted due to haptic device failure/timeout. Call ~/proceed to reset.");
      return;
    }

    if (state_ == TeleopState::HOMING) {
      run_homing_logic();
      return;
    }

    if (state_ == TeleopState::REGISTRATION) {
      publish_zero_velocity();
      if (auto_register_rcm_ && !rcm_registered_) {
        try_auto_register_rcm();
      }
      return;
    }

    if (state_ == TeleopState::CLUTCHED) {
      publish_zero_velocity();
      return;
    }

    if (state_ == TeleopState::ACTIVE) {
      run_active_mapping();
    }
  }

  void reset_command_limiter()
  {
    previous_linear_velocity_.setZero();
    previous_angular_velocity_.setZero();
    previous_linear_acceleration_.setZero();
    previous_angular_acceleration_.setZero();
    command_limiter_initialized_ = false;
  }

  Eigen::Vector3d limit_vector_rate(
    const Eigen::Vector3d & desired_velocity,
    Eigen::Vector3d & previous_velocity,
    Eigen::Vector3d & previous_acceleration,
    double max_acceleration,
    double max_jerk)
  {
    const double dt = 1.0 / update_rate_;
    if (!command_limiter_initialized_) {
      // A newly engaged clutch always ramps from rest.
      previous_linear_velocity_.setZero();
      previous_angular_velocity_.setZero();
      previous_linear_acceleration_.setZero();
      previous_angular_acceleration_.setZero();
      command_limiter_initialized_ = true;
    }

    Eigen::Vector3d desired_acceleration = (desired_velocity - previous_velocity) / dt;
    if (desired_acceleration.norm() > max_acceleration) {
      desired_acceleration = desired_acceleration.normalized() * max_acceleration;
    }

    Eigen::Vector3d acceleration_delta = desired_acceleration - previous_acceleration;
    const double max_acceleration_delta = max_jerk * dt;
    if (acceleration_delta.norm() > max_acceleration_delta) {
      acceleration_delta = acceleration_delta.normalized() * max_acceleration_delta;
    }

    Eigen::Vector3d limited_acceleration = previous_acceleration + acceleration_delta;
    if (limited_acceleration.norm() > max_acceleration) {
      limited_acceleration = limited_acceleration.normalized() * max_acceleration;
    }

    Eigen::Vector3d limited_velocity = previous_velocity + limited_acceleration * dt;
    const Eigen::Vector3d old_error = desired_velocity - previous_velocity;
    const Eigen::Vector3d new_error = desired_velocity - limited_velocity;
    if (old_error.dot(new_error) <= 0.0) {
      // Do not oscillate around a settled command because of stored acceleration.
      limited_velocity = desired_velocity;
      limited_acceleration = (limited_velocity - previous_velocity) / dt;
    }

    previous_velocity = limited_velocity;
    previous_acceleration = limited_acceleration;
    return limited_velocity;
  }

  void publish_zero_velocity()
  {
    reset_command_limiter();
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->get_clock()->now();
    twist_msg.header.frame_id = robot_base_frame_;
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;
    twist_pub_->publish(twist_msg);
  }

  void run_homing_logic()
  {
    if (positioning_mode_ == "haptic_jog") {
      if (haptic_positioning_active_) {
        run_active_mapping(
          false, positioning_max_linear_vel_, positioning_max_angular_vel_);
        return;
      }
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "HOMING: Hold Button 1 to jog without RCM, align the red tip/cyan shaft to "
        "the orange marker, release, wait for the arm to settle, then press Button 2.");
    } else if (positioning_mode_ == "physical_guiding") {
      const rclcpp::Time now = this->get_clock()->now();
      if (!guiding_controller_released_ && !controller_switch_pending_ &&
        (now - last_controller_switch_attempt_).seconds() >= 1.0)
      {
        request_arm_controller_switch(false);
      }
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "HOMING: Wait for the arm controller to become inactive, then use the Franka "
        "guiding buttons. Release the arm and call ~/proceed when stationary.");
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "HOMING: Robot positioning is fixed. Call ~/proceed to capture the current pose.");
    }
    publish_zero_velocity();
  }

  void run_active_mapping(
    bool enforce_rcm = true, double linear_velocity_limit = -1.0,
    double angular_velocity_limit = -1.0)
  {
    if (linear_velocity_limit <= 0.0) {
      linear_velocity_limit = max_linear_vel_;
    }
    if (angular_velocity_limit <= 0.0) {
      angular_velocity_limit = max_angular_vel_;
    }
    // Lookup current haptic pointer transform and current robot transform
    geometry_msgs::msg::TransformStamped haptic_tf;
    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      haptic_tf = tf_buffer_->lookupTransform(
        haptic_base_frame_, haptic_ee_frame_,
        tf2::TimePointZero);
      robot_tf =
        tf_buffer_->lookupTransform(robot_base_frame_, robot_ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "SAFETY HALT: Transform lookup failed in ACTIVE loop: %s. Halting robot!",
        ex.what());
      state_ = TeleopState::SAFETY_HALT;
      publish_zero_velocity();
      return;
    }

    // Check haptic transform freshness
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time haptic_time = haptic_tf.header.stamp;
    double age = (now - haptic_time).seconds();
    if (age > haptic_timeout_) {
      RCLCPP_ERROR(
        this->get_logger(), "SAFETY HALT: Haptic transform is stale (age: %.3f s, timeout: %.3f s). Halting robot!", age,
        haptic_timeout_);
      state_ = TeleopState::SAFETY_HALT;
      publish_zero_velocity();
      return;
    }

    const double robot_age = (now - rclcpp::Time(robot_tf.header.stamp)).seconds();
    const double joint_state_age = have_recent_joint_state_ ?
      (now - last_joint_state_receipt_time_).seconds() : std::numeric_limits<double>::infinity();
    if (robot_age > robot_state_timeout_ || robot_age < -0.01 ||
      joint_state_age > robot_state_timeout_)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "SAFETY HALT: Robot feedback is stale/time-invalid "
        "(TF age: %.3f s, joint-state age: %.3f s).",
        robot_age, joint_state_age);
      state_ = TeleopState::SAFETY_HALT;
      publish_zero_velocity();
      return;
    }

    Eigen::Vector3d haptic_pos(
      haptic_tf.transform.translation.x,
      haptic_tf.transform.translation.y,
      haptic_tf.transform.translation.z
    );

    Eigen::Vector3d p_curr(
      robot_tf.transform.translation.x,
      robot_tf.transform.translation.y,
      robot_tf.transform.translation.z
    );

    Eigen::Quaterniond q_curr(
      robot_tf.transform.rotation.w,
      robot_tf.transform.rotation.x,
      robot_tf.transform.rotation.y,
      robot_tf.transform.rotation.z
    );

    // 1. Calculate displacement from start of active cycle
    Eigen::Vector3d raw_disp = haptic_pos - haptic_start_pos_;

    // 2. Apply deadband threshold
    Eigen::Vector3d deadbanded_disp = raw_disp;
    double d_norm = raw_disp.norm();
    if (d_norm < deadband_position_) {
      deadbanded_disp.setZero();
    } else {
      deadbanded_disp = (1.0 - deadband_position_ / d_norm) * raw_disp;
    }

    // 3. Apply Butterworth filter to suppress user tremor
    Eigen::Vector3d filtered_disp = butterworth_filter_.filter(deadbanded_disp);

    // Transform haptic displacement into robot base frame
    Eigen::Vector3d mapped_disp = R_h_r_ * filtered_disp;

    Eigen::Vector3d cmd_pos;
    Eigen::Quaterniond cmd_q;

    if (enforce_rcm && use_rcm_) {
      // 4. Command a scaled Cartesian tool-tip displacement. Haptic orientation is
      // deliberately ignored; only the clutch-relative haptic position is used.
      const Eigen::Vector3d desired_tip =
        robot_start_tip_pos_ + tip_position_scale_ * mapped_disp;
      if (!desired_tip.allFinite()) {
        RCLCPP_ERROR(
          this->get_logger(), "SAFETY HALT: Non-finite Cartesian tip target detected.");
        state_ = TeleopState::SAFETY_HALT;
        reset_command_limiter();
        publish_zero_velocity();
        return;
      }

      // 5. Project the free Cartesian tip target into the permitted trocar cone and
      // insertion interval, then reconstruct the unique RCM-valid flange position.
      auto target = phantom_panda_teleop::cartesianTipTargetWithRcm(
        desired_tip, rcm_position_, rcm_start_shaft_axis_, tool_length_,
        min_insertion_depth_, max_insertion_depth_, max_tilt_angle_);

      // A point at the RCM has no direction. Blend the complete shaft direction away
      // from the clutch-time axis near the cone apex so neither tilt nor azimuth can
      // jump for an arbitrarily small lateral hand displacement.
      const Eigen::Vector3d blended_axis =
        phantom_panda_teleop::blendShaftDirectionNearApex(
        rcm_start_shaft_axis_, target.shaft_axis, target.insertion,
        tip_direction_transition_depth_);
      target.shaft_axis = blended_axis;
      target.tip_position = rcm_position_ - target.insertion * blended_axis;
      target.flange_position =
        rcm_position_ + (tool_length_ - target.insertion) * blended_axis;
      cmd_pos = target.flange_position;

      // 6. The RCM geometry determines the tool axis. Axial roll is not observable from
      // a point target, so preserve the roll present when the clutch was engaged.
      const Eigen::Vector3d z_ee = -target.shaft_axis;
      const Eigen::Quaterniond shaft_alignment =
        Eigen::Quaterniond::FromTwoVectors(rcm_start_z_axis_, z_ee);
      cmd_q = shaft_alignment * robot_start_rot_;
      cmd_q.normalize();
    } else {
      // Position-only direct Cartesian mapping. Haptic orientation is intentionally ignored.
      cmd_pos = robot_start_pos_ + k_linear_ * mapped_disp;
      cmd_q = robot_start_rot_;
    }

    // 7. Closed-loop Proportional Controller to generate TwistStamped velocity commands
    if (!cmd_pos.allFinite() || !cmd_q.coeffs().allFinite() ||
      !p_curr.allFinite() || !q_curr.coeffs().allFinite())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "SAFETY HALT: Non-finite Cartesian state or target detected.");
      state_ = TeleopState::SAFETY_HALT;
      reset_command_limiter();
      publish_zero_velocity();
      return;
    }

    q_curr.normalize();
    cmd_q.normalize();
    Eigen::Vector3d p_err = cmd_pos - p_curr;
    Eigen::Quaterniond q_err = cmd_q * q_curr.conjugate();
    q_err.normalize();
    if (q_err.w() < 0.0) {
      q_err.coeffs() *= -1.0; // q and -q are identical; choose the shortest rotation
    }
    const Eigen::Vector3d q_err_vec(q_err.x(), q_err.y(), q_err.z());
    const double q_err_vec_norm = q_err_vec.norm();
    Eigen::Vector3d rot_err = Eigen::Vector3d::Zero();
    if (q_err_vec_norm > 1e-9) {
      const double angle = 2.0 * std::atan2(q_err_vec_norm, q_err.w());
      rot_err = angle * q_err_vec / q_err_vec_norm;
    }

    Eigen::Vector3d v_cmd = kp_pos_ * p_err;
    Eigen::Vector3d w_cmd = kp_rot_ * rot_err;

    // Filter output velocity commands to suppress hand tremors and command spikes
    Eigen::Vector3d v_cmd_filtered = twist_linear_filter_.filter(v_cmd);
    Eigen::Vector3d w_cmd_filtered = twist_angular_filter_.filter(w_cmd);

    // p_err and rot_err are expressed in robot_base_frame_. Keep the Cartesian command
    // in that same frame. MoveIt Servo Humble uses robot_link_command_frame from its
    // configuration; converting these values to the EE frame while Servo is consuming
    // them as planning-frame components makes the physical arm move away from the target.
    Eigen::Vector3d v_cmd_base = v_cmd_filtered;
    Eigen::Vector3d w_cmd_base = w_cmd_filtered;

    // Velocity limits
    if (v_cmd_base.norm() > linear_velocity_limit) {
      v_cmd_base = v_cmd_base.normalized() * linear_velocity_limit;
    }
    if (w_cmd_base.norm() > angular_velocity_limit) {
      w_cmd_base = w_cmd_base.normalized() * angular_velocity_limit;
    }

    // Enforce acceleration and jerk bounds after filtering.
    v_cmd_base = limit_vector_rate(
      v_cmd_base, previous_linear_velocity_,
      previous_linear_acceleration_, max_linear_accel_, max_linear_jerk_);
    w_cmd_base = limit_vector_rate(
      w_cmd_base, previous_angular_velocity_,
      previous_angular_acceleration_, max_angular_accel_, max_angular_jerk_);

    const double command_norm = v_cmd_base.norm() + w_cmd_base.norm();
    if (command_norm > 1e-4) {
      const rclcpp::Time command_now = this->get_clock()->now();
      const double servo_command_age = have_servo_joint_command_ ?
        (command_now - last_servo_joint_command_time_).seconds() :
        std::numeric_limits<double>::infinity();
      const double controller_state_age = have_controller_state_ ?
        (command_now - last_controller_state_time_).seconds() :
        std::numeric_limits<double>::infinity();

      const bool diagnostic_grace_elapsed =
        (command_now - active_mapping_start_time_).seconds() > command_chain_timeout_;
      if (diagnostic_grace_elapsed && servo_command_age > command_chain_timeout_) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Command-chain fault: nonzero Cartesian commands reach %s, but Servo has not "
          "published a recent joint trajectory on %s (age %.3f s). Check Servo status, "
          "collision/singularity state, and joint feedback.",
          servo_twist_topic_.c_str(), servo_joint_command_topic_.c_str(), servo_command_age);
      } else if (diagnostic_grace_elapsed && controller_state_age > command_chain_timeout_) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Command-chain fault: Servo publishes joint trajectories, but no recent controller "
          "state is present on %s (age %.3f s). Check fr3_arm_controller and the FCI connection.",
          controller_state_topic_.c_str(), controller_state_age);
      } else if (max_controller_position_error_ > 0.05) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Arm controller tracking error is large (max joint position error %.3f rad). "
          "Servo commands are reaching the controller, but hardware is not following them.",
          max_controller_position_error_);
      } else if (max_controller_velocity_error_ > 0.10) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Arm controller velocity tracking error is large (max %.3f rad/s). "
          "Short-horizon Servo targets are reaching the controller, but their requested "
          "joint motion is not being realized.",
          max_controller_velocity_error_);
      }
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Tracking | P_err: [%.3f, %.3f, %.3f], R_err: [%.3f, %.3f, %.3f]",
      p_err.x(), p_err.y(), p_err.z(), rot_err.x(), rot_err.y(), rot_err.z());

    // 8. Publish TwistStamped to MoveIt Servo
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->get_clock()->now();
    twist_msg.header.frame_id = robot_base_frame_;
    twist_msg.twist.linear.x = v_cmd_base.x();
    twist_msg.twist.linear.y = v_cmd_base.y();
    twist_msg.twist.linear.z = v_cmd_base.z();
    twist_msg.twist.angular.x = w_cmd_base.x();
    twist_msg.twist.angular.y = w_cmd_base.y();
    twist_msg.twist.angular.z = w_cmd_base.z();
    twist_pub_->publish(twist_msg);
  }

  // Node Variables
  double update_rate_;
  std::string haptic_base_frame_;
  std::string haptic_ee_frame_;
  std::string robot_base_frame_;
  std::string robot_ee_frame_;
  std::string robot_tool_tip_frame_;
  double tip_position_scale_;
  double max_tilt_angle_;
  double tip_direction_transition_depth_;
  double deadband_position_;
  double cutoff_freq_;
  double tool_length_;
  double min_insertion_depth_;
  double max_insertion_depth_;
  double registration_insertion_depth_;
  double rcm_hole_diameter_;
  double shaft_marker_extension_;
  double rcm_warning_error_;
  std::string custom_tool_state_topic_;

  // Closed loop parameters
  double kp_pos_;
  double kp_rot_;
  double max_linear_vel_;
  double max_angular_vel_;
  double max_linear_accel_;
  double max_angular_accel_;
  double max_linear_jerk_;
  double max_angular_jerk_;

  TeleopState state_;
  Eigen::Vector3d rcm_position_;
  Eigen::Matrix3d R_h_r_;
  Vector3ButterworthFilter butterworth_filter_;

  // Clutch start offsets
  Eigen::Vector3d haptic_start_pos_;

  // Direct Cartesian tracking start offsets
  Eigen::Vector3d robot_start_pos_;
  Eigen::Vector3d robot_start_tip_pos_;
  Eigen::Quaterniond robot_start_rot_;
  Eigen::Vector3d rcm_start_z_axis_;
  Eigen::Vector3d rcm_start_shaft_axis_;
  bool use_rcm_;
  bool rcm_registered_;
  double k_linear_;
  double haptic_timeout_;
  double robot_state_timeout_;
  double command_chain_timeout_;
  std::string joy_topic_;
  std::string joint_state_topic_;
  std::string servo_twist_topic_;
  std::string servo_status_topic_;
  std::string servo_start_service_;
  std::string servo_joint_command_topic_;
  std::string controller_state_topic_;
  std::string positioning_mode_;
  bool start_in_registration_;
  bool auto_register_rcm_;
  std::string controller_switch_service_;
  std::string arm_controller_name_;
  double positioning_max_linear_vel_;
  double positioning_max_angular_vel_;
  double registration_joint_velocity_tolerance_;
  double registration_settle_time_;
  std::vector<double> joint_velocity_halt_limits_;
  std::array<double, 7> latest_joint_velocities_;
  rclcpp::Time last_joint_state_receipt_time_;
  rclcpp::Time last_joint_motion_time_;
  rclcpp::Time last_controller_switch_attempt_;
  rclcpp::Time last_auto_registration_attempt_;
  bool have_recent_joint_state_;
  rclcpp::Time last_servo_joint_command_time_;
  rclcpp::Time last_controller_state_time_;
  rclcpp::Time active_mapping_start_time_;
  bool have_servo_joint_command_;
  bool have_controller_state_;
  double max_controller_position_error_;
  double max_controller_velocity_error_;
  bool haptic_positioning_active_;
  bool button1_pressed_;
  bool controller_switch_pending_;
  bool guiding_controller_released_;

  // Homing variables
  double homing_timer_;
  Eigen::Vector3d homing_start_pos_;
  Eigen::Quaterniond homing_start_rot_;

  // Placeholder state for the future custom tool actuator.
  bool custom_tool_closed_;
  int prev_joy_button1_;
  int prev_joy_button2_;

  // TF buffers
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers / Subscribers / Servers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    servo_joint_command_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr teleop_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr custom_tool_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rcm_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rcm_error_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr register_rcm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr proceed_srv_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    controller_switch_client_;

  // MoveIt Servo start state and client
  bool servo_started_;
  bool servo_start_request_pending_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;

  bool maximize_collision_thresholds_;
  rclcpp::Client<franka_msgs::srv::SetFullCollisionBehavior>::SharedPtr
    set_collision_behavior_client_;

  // Filters for smoothing final output twist commands
  Vector3ButterworthFilter twist_linear_filter_;
  Vector3ButterworthFilter twist_angular_filter_;
  Eigen::Vector3d previous_linear_velocity_;
  Eigen::Vector3d previous_angular_velocity_;
  Eigen::Vector3d previous_linear_acceleration_;
  Eigen::Vector3d previous_angular_acceleration_;
  bool command_limiter_initialized_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PhantomPandaTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
