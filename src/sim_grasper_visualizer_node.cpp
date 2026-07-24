#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
constexpr double kOpenJawAngle = 0.55;
constexpr char kToolTipFrame[] = "biorob_tool_tip";
}  // namespace

class SimGrasperVisualizer : public rclcpp::Node
{
public:
  SimGrasperVisualizer()
  : Node("sim_grasper_visualizer")
  {
    declare_parameter("tool_state_topic", "/phantom_panda_teleop_node/custom_tool_closed");
    declare_parameter("joint_state_topic", "/joint_states");
    declare_parameter("marker_topic", "~/markers");

    const auto state_topic = get_parameter("tool_state_topic").as_string();
    const auto joint_state_topic = get_parameter("joint_state_topic").as_string();
    const auto marker_topic = get_parameter("marker_topic").as_string();

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    state_subscription_ = create_subscription<std_msgs::msg::Bool>(
      state_topic, state_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        closed_ = message->data;
        publish();
      });
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::QoS(10));
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, state_qos);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publish();});
    publish();
    RCLCPP_INFO(
      get_logger(), "Simulated grasper attached to %s; listening on %s.",
      kToolTipFrame, state_topic.c_str());
  }

private:
  void publish()
  {
    const auto stamp = now();
    const double jaw_angle = closed_ ? 0.0 : kOpenJawAngle;

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = {"biorob_left_jaw_joint", "biorob_right_jaw_joint"};
    joints.position = {jaw_angle, -jaw_angle};
    joint_state_publisher_->publish(joints);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker hinge;
    hinge.header.frame_id = kToolTipFrame;
    // This is an RViz-only state indicator.  A zero timestamp asks RViz for
    // the latest available tool-tip transform instead of an exact transform at
    // this node's wall-clock instant.  The arm/controller TF stream can lag a
    // just-published joint state by one cycle, which otherwise causes transient
    // "would require extrapolation" errors and flickering markers.
    hinge.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    hinge.ns = "simulated_grasper";
    hinge.id = 0;
    hinge.type = visualization_msgs::msg::Marker::SPHERE;
    hinge.action = visualization_msgs::msg::Marker::ADD;
    hinge.scale.x = 0.012;
    hinge.scale.y = 0.012;
    hinge.scale.z = 0.012;
    hinge.color.a = 1.0F;
    hinge.color.r = closed_ ? 0.95F : 0.10F;
    hinge.color.g = closed_ ? 0.15F : 0.85F;
    hinge.color.b = 0.10F;
    markers.markers.push_back(hinge);

    visualization_msgs::msg::Marker label;
    label.header = hinge.header;
    label.ns = hinge.ns;
    label.id = 1;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.z = 0.070;
    label.scale.z = 0.018;
    label.color = hinge.color;
    label.text = closed_ ? "GRIPPER: CLOSED" : "GRIPPER: OPEN";
    markers.markers.push_back(label);
    marker_publisher_->publish(markers);
  }

  bool closed_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimGrasperVisualizer>());
  rclcpp::shutdown();
  return 0;
}
