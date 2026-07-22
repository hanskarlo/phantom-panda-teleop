"""
Launch MoveIt planning and RViz for the BioRob FR3 tool model.

This is deliberately separate from the streaming teleoperation launch.  The
physical-hardware path uses an effort-based JointTrajectoryController, while
the fake-hardware path uses position commands so GenericSystem can mirror the
commanded trajectory into joint state.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    package_name = "phantom_panda_teleop"
    package_share = get_package_share_directory(package_name)

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    headless = LaunchConfiguration("headless")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description_command = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            os.path.join(package_share, "urdf", "biorob_panda.urdf.xacro"),
            " hand:=false",
            " ee_id:=none",
            " robot_ip:=",
            robot_ip,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
            " ros2_control:=true",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_command, value_type=str)
    }

    robot_description_semantic_command = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            os.path.join(package_share, "urdf", "biorob_panda.srdf.xacro"),
            " hand:=false",
            " ee_id:=none",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_command, value_type=str
        )
    }

    robot_description_kinematics = load_yaml(package_name, "config/kinematics.yaml")
    robot_description_planning = load_yaml(package_name, "config/joint_limits.yaml")

    ompl_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_config["move_group"].update(
        load_yaml(package_name, "config/ompl_planning.yaml")
    )
    moveit_simple_controllers = load_yaml(
        package_name, "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_config,
        ],
        condition=UnlessCondition(headless),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controller_file = PythonExpression(
        [
            "'fr3_ros_controllers_moveit_fake.yaml' if '",
            use_fake_hardware,
            "'.lower() == 'true' else 'fr3_ros_controllers_moveit.yaml'",
        ]
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", controller_file]
    )
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        remappings=[("joint_states", "/franka/joint_states")],
        output={"stdout": "screen", "stderr": "screen"},
        on_exit=Shutdown(),
    )

    load_arm_controllers = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "joint_state_broadcaster",
            "fr3_arm_controller",
            "--activate-as-group",
            "--controller-manager-timeout",
            "60",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    franka_robot_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "franka_robot_state_broadcaster",
            "--controller-manager-timeout",
            "60",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"source_list": ["/franka/joint_states"], "rate": 100}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="dont-care",
                description=(
                    "Robot hostname or IP; required when use_fake_hardware is false"
                ),
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use ros2_control GenericSystem instead of the physical FR3",
            ),
            DeclareLaunchArgument(
                "fake_sensor_commands",
                default_value="false",
                description="Mirror fake sensor commands (fake hardware only)",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Start planning and control without RViz",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(package_share, "config", "moveit.rviz"),
                description="RViz configuration file",
            ),
            robot_state_publisher,
            ros2_control,
            joint_state_publisher,
            franka_robot_state_broadcaster,
            load_arm_controllers,
            move_group,
            rviz,
        ]
    )
