#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// Butterworth low-pass filter implementation for tremor suppression
class ButterworthFilter {
public:
  ButterworthFilter() : b0(0), b1(0), b2(0), a1(0), a2(0), x1(0), x2(0), y1(0), y2(0), initialized(false) {}

  void setup(double cutoff_freq, double sample_rate) {
    double K = std::tan(M_PI * cutoff_freq / sample_rate);
    double D = K * K + std::sqrt(2.0) * K + 1.0;
    b0 = K * K / D;
    b1 = 2.0 * K * K / D;
    b2 = K * K / D;
    a1 = 2.0 * (K * K - 1.0) / D;
    a2 = (K * K - std::sqrt(2.0) * K + 1.0) / D;
    reset();
  }

  void reset() {
    x1 = x2 = 0.0;
    y1 = y2 = 0.0;
    initialized = false;
  }

  double filter(double input) {
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
class Vector3ButterworthFilter {
public:
  void setup(double cutoff_freq, double sample_rate) {
    filters[0].setup(cutoff_freq, sample_rate);
    filters[1].setup(cutoff_freq, sample_rate);
    filters[2].setup(cutoff_freq, sample_rate);
  }
  void reset() {
    filters[0].reset();
    filters[1].reset();
    filters[2].reset();
  }
  Eigen::Vector3d filter(const Eigen::Vector3d &input) {
    return Eigen::Vector3d(
      filters[0].filter(input.x()),
      filters[1].filter(input.y()),
      filters[2].filter(input.z())
    );
  }
private:
  ButterworthFilter filters[3];
};

enum class TeleopState {
  HOMING = 0,
  REGISTRATION = 1,
  CLUTCHED = 2,
  ACTIVE = 3,
  SAFETY_HALT = 4
};

class PhantomPandaTeleopNode : public rclcpp::Node {
public:
  PhantomPandaTeleopNode() : Node("phantom_panda_teleop_node") {
    // Declare and retrieve parameters
    this->declare_parameter("update_rate", 100.0);
    this->declare_parameter("haptic_base_frame", "touch_x_base");
    this->declare_parameter("haptic_ee_frame", "touch_x_ee");
    this->declare_parameter("robot_base_frame", "fr3_link0");
    this->declare_parameter("robot_ee_frame", "fr3_hand");
    this->declare_parameter("k_theta", 2.0); // scale factor for azimuth
    this->declare_parameter("k_phi", 2.0);   // scale factor for elevation
    this->declare_parameter("k_r", 0.5);     // scale factor for insertion depth
    this->declare_parameter("k_roll", 1.0);  // scale factor for wrist roll
    this->declare_parameter("deadband_position", 0.0005); // 0.5 mm deadband
    this->declare_parameter("cutoff_freq", 5.0); // 5 Hz tremor low-pass cutoff
    this->declare_parameter("rcm_x", 0.4);
    this->declare_parameter("rcm_y", 0.0);
    this->declare_parameter("rcm_z", 0.2);
    this->declare_parameter("tool_length", 0.25); // 25 cm tool length
    this->declare_parameter("gripper_action_name", "/franka_gripper/gripper_action");
    this->declare_parameter("gripper_open_width", 0.08); // 8 cm maximum open width
    this->declare_parameter("gripper_close_width", 0.00);
    this->declare_parameter("gripper_force", 10.0); // 10 N grip force

    // Closed-loop proportional tracking gains
    this->declare_parameter("kp_pos", 12.0);
    this->declare_parameter("kp_rot", 8.0);
    this->declare_parameter("max_linear_vel", 0.15); // max Cartesian linear speed 15 cm/s
    this->declare_parameter("max_angular_vel", 0.5); // max Cartesian angular speed 0.5 rad/s

    this->get_parameter("update_rate", update_rate_);
    this->get_parameter("haptic_base_frame", haptic_base_frame_);
    this->get_parameter("haptic_ee_frame", haptic_ee_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("robot_ee_frame", robot_ee_frame_);
    this->get_parameter("k_theta", k_theta_);
    this->get_parameter("k_phi", k_phi_);
    this->get_parameter("k_r", k_r_);
    this->get_parameter("k_roll", k_roll_);
    this->get_parameter("deadband_position", deadband_position_);
    this->get_parameter("cutoff_freq", cutoff_freq_);
    this->get_parameter("tool_length", tool_length_);
    this->get_parameter("gripper_action_name", gripper_action_name_);
    this->get_parameter("gripper_open_width", gripper_open_width_);
    this->get_parameter("gripper_close_width", gripper_close_width_);
    this->get_parameter("gripper_force", gripper_force_);
    this->get_parameter("kp_pos", kp_pos_);
    this->get_parameter("kp_rot", kp_rot_);
    this->get_parameter("max_linear_vel", max_linear_vel_);
    this->get_parameter("max_angular_vel", max_angular_vel_);

    // Set initial RCM coordinates
    rcm_position_ = Eigen::Vector3d(
      this->get_parameter("rcm_x").as_double(),
      this->get_parameter("rcm_y").as_double(),
      this->get_parameter("rcm_z").as_double()
    );

    // Setup filtering
    butterworth_filter_.setup(cutoff_freq_, update_rate_);

    // Setup TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup Subscribers & Publishers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/geomagic_touch_x/joy", rclcpp::SensorDataQoS(), std::bind(&PhantomPandaTeleopNode::joy_callback, this, std::placeholders::_1));

    // MoveIt Servo Cartesian command input publisher
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    // Setup calibration services
    register_rcm_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/register_rcm", std::bind(&PhantomPandaTeleopNode::register_rcm_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Setup gripper action client
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      this, gripper_action_name_);

    // Initialize state
    state_ = TeleopState::HOMING;
    homing_timer_ = 0.0;
    gripper_open_ = true;
    prev_joy_button2_ = 0;
    servo_started_ = false;
    start_servo_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    // Run core control loop
    double loop_period_ms = 1000.0 / update_rate_;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(loop_period_ms),
      std::bind(&PhantomPandaTeleopNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "BioRob teleop node initialized in HOMING mode. RCM Point: [%.3f, %.3f, %.3f]",
                rcm_position_.x(), rcm_position_.y(), rcm_position_.z());
  }

private:
  // Service to dynamically register Trocar RCM point
  void register_rcm_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    // Lookup current robot pose
    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      robot_tf = tf_buffer_->lookupTransform(robot_base_frame_, robot_ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      response->success = false;
      response->message = "Failed to lookup robot transform: " + std::string(ex.what());
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

    // Compute Trojan center (RCM) assuming the tool tip is placed exactly in the center of the trocar.
    // The tool vector extends along the end-effector's local Z-axis (pointing forward).
    Eigen::Vector3d z_ee = q_ee.toRotationMatrix().col(2);
    rcm_position_ = p_ee + tool_length_ * z_ee;

    RCLCPP_INFO(this->get_logger(), "Registered RCM trocar center at: [%.3f, %.3f, %.3f]",
                rcm_position_.x(), rcm_position_.y(), rcm_position_.z());

    response->success = true;
    response->message = "Trocar / RCM point registered successfully.";

    // If registered, transition from REGISTRATION to CLUTCHED
    if (state_ == TeleopState::REGISTRATION) {
      state_ = TeleopState::CLUTCHED;
      RCLCPP_INFO(this->get_logger(), "Transitioned from REGISTRATION to CLUTCHED state.");
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size() < 2) return;

    int button1 = msg->buttons[0]; // Front button (Clutch deadman switch)
    int button2 = msg->buttons[1]; // Rear button (Gripper toggle)

    // Handle deadman clutch logic
    if (state_ == TeleopState::CLUTCHED && button1 == 1) {
      // Transition to ACTIVE
      if (initialize_active_mapping()) {
        state_ = TeleopState::ACTIVE;
        RCLCPP_INFO(this->get_logger(), "Clutch engaged -> ACTIVE state.");
      }
    } else if (state_ == TeleopState::ACTIVE && button1 == 0) {
      // Transition back to CLUTCHED
      state_ = TeleopState::CLUTCHED;
      RCLCPP_INFO(this->get_logger(), "Clutch disengaged -> CLUTCHED state. Freezing robot.");
    }

    // Handle gripper toggle action on rising edge of Button 2
    if (button2 == 1 && prev_joy_button2_ == 0) {
      toggle_gripper();
    }
    prev_joy_button2_ = button2;
  }

  bool initialize_active_mapping() {
    // Lookup starting transforms to register clutch offsets
    geometry_msgs::msg::TransformStamped haptic_tf;
    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      haptic_tf = tf_buffer_->lookupTransform(haptic_base_frame_, haptic_ee_frame_, tf2::TimePointZero);
      robot_tf = tf_buffer_->lookupTransform(robot_base_frame_, robot_ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not initialize ACTIVE mapping: %s", ex.what());
      return false;
    }

    // Register initial haptic coordinates
    haptic_start_pos_ = Eigen::Vector3d(
      haptic_tf.transform.translation.x,
      haptic_tf.transform.translation.y,
      haptic_tf.transform.translation.z
    );

    haptic_start_rot_ = Eigen::Quaterniond(
      haptic_tf.transform.rotation.w,
      haptic_tf.transform.rotation.x,
      haptic_tf.transform.rotation.y,
      haptic_tf.transform.rotation.z
    );

    // Register initial robot state relative to registered RCM
    Eigen::Vector3d p_ee_start(
      robot_tf.transform.translation.x,
      robot_tf.transform.translation.y,
      robot_tf.transform.translation.z
    );

    // Compute relative distance from RCM
    Eigen::Vector3d u_start = (p_ee_start - rcm_position_);
    double lambda_ee_start = u_start.norm();
    if (lambda_ee_start < 1e-4) {
      RCLCPP_ERROR(this->get_logger(), "Robot EE start position coincides with RCM! Check calibrations.");
      return false;
    }
    u_start.normalize();

    // Insertion depth initial value
    r_start_ = tool_length_ - lambda_ee_start;

    // Convert starting tool axis unit vector to spherical angles (azimuth theta, elevation phi)
    // u = [sin(phi)*cos(theta), sin(phi)*sin(theta), -cos(phi)]
    phi_start_ = std::acos(-u_start.z());
    theta_start_ = std::atan2(u_start.y(), u_start.x());

    // Filter reset
    butterworth_filter_.reset();

    // Log tracking target info
    RCLCPP_INFO(this->get_logger(), "Teleop values locked: theta0=%.2f deg, phi0=%.2f deg, r0=%.3f m",
                theta_start_ * 180.0 / M_PI, phi_start_ * 180.0 / M_PI, r_start_);
    return true;
  }

  void toggle_gripper() {
    if (!gripper_action_client_->action_server_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Gripper action server not available.");
      return;
    }

    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    if (gripper_open_) {
      goal_msg.command.position = gripper_close_width_;
      goal_msg.command.max_effort = gripper_force_;
      RCLCPP_INFO(this->get_logger(), "Sending Gripper CLOSE command.");
    } else {
      goal_msg.command.position = gripper_open_width_;
      goal_msg.command.max_effort = gripper_force_;
      RCLCPP_INFO(this->get_logger(), "Sending Gripper OPEN command.");
    }

    // Toggle target state
    gripper_open_ = !gripper_open_;

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = [this](const auto &result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Gripper command completed successfully.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Gripper command failed or was canceled.");
      }
    };
    gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void start_moveit_servo() {
    if (servo_started_) return;

    if (!start_servo_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for /servo_node/start_servo service...");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    servo_started_ = true; // prevent duplicate parallel calls
    
    start_servo_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "Successfully started MoveIt Servo.");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to start MoveIt Servo: %s", response->message.c_str());
          servo_started_ = false; // retry on next control loop iteration if failed
        }
      });
  }

  void control_loop() {
    start_moveit_servo();

    if (state_ == TeleopState::HOMING) {
      run_homing_logic();
      return;
    }

    if (state_ == TeleopState::REGISTRATION) {
      publish_zero_velocity();
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

  void publish_zero_velocity() {
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

  void run_homing_logic() {
    // Smoothly guide robot from current pose to initial safe start pose above box
    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      robot_tf = tf_buffer_->lookupTransform(robot_base_frame_, robot_ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for robot transforms: %s", ex.what());
      return;
    }

    // Predefined home position
    Eigen::Vector3d home_pos(0.4, 0.0, 0.45);
    // Home orientation: Z-axis pointing straight down (rotation of 180 degrees around X)
    Eigen::Quaterniond home_rot(0.0, 1.0, 0.0, 0.0);

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

    // If first iteration of homing, record start pose
    if (homing_timer_ == 0.0) {
      homing_start_pos_ = p_curr;
      homing_start_rot_ = q_curr;
      RCLCPP_INFO(this->get_logger(), "Starting homing trajectory interpolation...");
    }

    homing_timer_ += (1.0 / update_rate_);
    double duration = 4.0; // 4 seconds homing duration
    double t = std::min(homing_timer_ / duration, 1.0);

    // Linear interpolation of position and spherical linear interpolation (slerp) of orientation
    Eigen::Vector3d target_pos = homing_start_pos_ + t * (home_pos - homing_start_pos_);
    Eigen::Quaterniond target_rot = homing_start_rot_.slerp(t, home_rot);

    // Proportional control towards target trajectory
    Eigen::Vector3d p_err = target_pos - p_curr;
    Eigen::Quaterniond q_err = target_rot * q_curr.inverse();
    Eigen::AngleAxisd angle_axis(q_err);
    Eigen::Vector3d rot_err = angle_axis.angle() * angle_axis.axis();

    Eigen::Vector3d v_cmd = kp_pos_ * p_err;
    Eigen::Vector3d w_cmd = kp_rot_ * rot_err;

    // Velocity limits
    if (v_cmd.norm() > max_linear_vel_) v_cmd = v_cmd.normalized() * max_linear_vel_;
    if (w_cmd.norm() > max_angular_vel_) w_cmd = w_cmd.normalized() * max_angular_vel_;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->get_clock()->now();
    twist_msg.header.frame_id = robot_base_frame_;
    twist_msg.twist.linear.x = v_cmd.x();
    twist_msg.twist.linear.y = v_cmd.y();
    twist_msg.twist.linear.z = v_cmd.z();
    twist_msg.twist.angular.x = w_cmd.x();
    twist_msg.twist.angular.y = w_cmd.y();
    twist_msg.twist.angular.z = w_cmd.z();
    twist_pub_->publish(twist_msg);

    if (t >= 1.0) {
      // Transition to registration mode
      state_ = TeleopState::REGISTRATION;
      RCLCPP_INFO(this->get_logger(), "Robot reached home posture. Transitioned to REGISTRATION. Please align tool and register RCM.");
    }
  }

  void run_active_mapping() {
    // Lookup current haptic pointer transform and current robot transform
    geometry_msgs::msg::TransformStamped haptic_tf;
    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      haptic_tf = tf_buffer_->lookupTransform(haptic_base_frame_, haptic_ee_frame_, tf2::TimePointZero);
      robot_tf = tf_buffer_->lookupTransform(robot_base_frame_, robot_ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Transform lookup failed in ACTIVE loop: %s", ex.what());
      return;
    }

    Eigen::Vector3d haptic_pos(
      haptic_tf.transform.translation.x,
      haptic_tf.transform.translation.y,
      haptic_tf.transform.translation.z
    );

    Eigen::Quaterniond haptic_rot(
      haptic_tf.transform.rotation.w,
      haptic_tf.transform.rotation.x,
      haptic_tf.transform.rotation.y,
      haptic_tf.transform.rotation.z
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

    // 4. Map displacements to Spherical Coordinates pivoting about registered trocar
    double theta = theta_start_ + k_theta_ * filtered_disp.x(); // Stylus Left/Right maps to azimuth
    double phi = phi_start_ + k_phi_ * filtered_disp.y();     // Stylus Up/Down maps to elevation
    double r = r_start_ + k_r_ * filtered_disp.z();         // Stylus In/Out maps to depth

    // Safety limits for trocar angles and insertion depths
    phi = std::clamp(phi, 0.0, 0.52); // Max 30 deg tilt to protect port boundaries
    r = std::clamp(r, 0.05, tool_length_ - 0.02); // Protect tool tip collisions and flange over-extension

    // 5. Reconstruct tool axis vector pointing from RCM towards end-effector
    Eigen::Vector3d u;
    u.x() = std::sin(phi) * std::cos(theta);
    u.y() = std::sin(phi) * std::sin(theta);
    u.z() = -std::cos(phi);

    // 6. Compute new Cartesian target for the robot end-effector flange
    double lambda_ee = tool_length_ - r;
    Eigen::Vector3d cmd_pos = rcm_position_ + lambda_ee * u;

    // 7. Compute target orientation matrix (Align flange Z-axis pointing along tool shaft)
    Eigen::Vector3d z_ee = -u; // Point along the shaft from EE to Trocar
    Eigen::Vector3d v_up(0.0, 0.0, 1.0); // reference frame alignment vector
    Eigen::Vector3d y_ee = z_ee.cross(v_up);
    if (y_ee.norm() < 1e-4) {
      // Singularity fallback (tool points exactly along vertical vector)
      y_ee = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else {
      y_ee.normalize();
    }
    Eigen::Vector3d x_ee = y_ee.cross(z_ee);
    x_ee.normalize();

    Eigen::Matrix3d R_cmd;
    R_cmd.col(0) = x_ee;
    R_cmd.col(1) = y_ee;
    R_cmd.col(2) = z_ee;

    // 8. Compute relative stylus roll and apply rotation around tool axis
    Eigen::Vector3d initial_pointer = haptic_start_rot_.toRotationMatrix().col(2);
    Eigen::Vector3d initial_side = haptic_start_rot_.toRotationMatrix().col(0);
    Eigen::Vector3d current_side = haptic_rot.toRotationMatrix().col(0);

    // Project current side onto plane perp to initial pointer
    Eigen::Vector3d side_proj = current_side - (current_side.dot(initial_pointer)) * initial_pointer;
    double roll_angle = 0.0;
    if (side_proj.norm() > 1e-4) {
      side_proj.normalize();
      double sin_roll = (initial_side.cross(side_proj)).dot(initial_pointer);
      double cos_roll = initial_side.dot(side_proj);
      roll_angle = std::atan2(sin_roll, cos_roll);
    }

    double cmd_roll = k_roll_ * roll_angle;
    Eigen::AngleAxisd roll_rot(cmd_roll, z_ee);
    Eigen::Quaterniond cmd_q(roll_rot * R_cmd);

    // 9. Closed-loop Proportional Controller to generate TwistStamped velocity commands
    Eigen::Vector3d p_err = cmd_pos - p_curr;
    Eigen::Quaterniond q_err = cmd_q * q_curr.inverse();
    Eigen::AngleAxisd angle_axis(q_err);
    Eigen::Vector3d rot_err = angle_axis.angle() * angle_axis.axis();

    Eigen::Vector3d v_cmd = kp_pos_ * p_err;
    Eigen::Vector3d w_cmd = kp_rot_ * rot_err;

    // Velocity limits
    if (v_cmd.norm() > max_linear_vel_) v_cmd = v_cmd.normalized() * max_linear_vel_;
    if (w_cmd.norm() > max_angular_vel_) w_cmd = w_cmd.normalized() * max_angular_vel_;

    // 10. Publish TwistStamped to MoveIt Servo
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->get_clock()->now();
    twist_msg.header.frame_id = robot_base_frame_;
    twist_msg.twist.linear.x = v_cmd.x();
    twist_msg.twist.linear.y = v_cmd.y();
    twist_msg.twist.linear.z = v_cmd.z();
    twist_msg.twist.angular.x = w_cmd.x();
    twist_msg.twist.angular.y = w_cmd.y();
    twist_msg.twist.angular.z = w_cmd.z();
    twist_pub_->publish(twist_msg);
  }

  // Node Variables
  double update_rate_;
  std::string haptic_base_frame_;
  std::string haptic_ee_frame_;
  std::string robot_base_frame_;
  std::string robot_ee_frame_;
  double k_theta_, k_phi_, k_r_, k_roll_;
  double deadband_position_;
  double cutoff_freq_;
  double tool_length_;
  std::string gripper_action_name_;
  double gripper_open_width_;
  double gripper_close_width_;
  double gripper_force_;

  // Closed loop parameters
  double kp_pos_;
  double kp_rot_;
  double max_linear_vel_;
  double max_angular_vel_;

  TeleopState state_;
  Eigen::Vector3d rcm_position_;
  Vector3ButterworthFilter butterworth_filter_;

  // Clutch start offsets
  Eigen::Vector3d haptic_start_pos_;
  Eigen::Quaterniond haptic_start_rot_;
  double theta_start_, phi_start_, r_start_;

  // Homing variables
  double homing_timer_;
  Eigen::Vector3d homing_start_pos_;
  Eigen::Quaterniond homing_start_rot_;

  // Gripper state
  bool gripper_open_;
  int prev_joy_button2_;

  // TF buffers
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers / Subscribers / Servers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr register_rcm_srv_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;

  // MoveIt Servo start state and client
  bool servo_started_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PhantomPandaTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
