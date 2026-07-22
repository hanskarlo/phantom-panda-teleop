import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Declare launch arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware (simulation) or physical robot'
    )
    controller_command_topic_arg = DeclareLaunchArgument(
        'controller_command_topic',
        default_value='/fr3_arm_controller/joint_trajectory',
        description='Joint-trajectory input of the active arm controller'
    )
    controller_state_topic_arg = DeclareLaunchArgument(
        'controller_state_topic',
        default_value='/fr3_arm_controller/controller_state',
        description='State feedback topic of the active arm controller'
    )

    # 1. Robot Description (URDF xacro)
    franka_xacro_file = os.path.join(
        get_package_share_directory('phantom_panda_teleop'),
        'urdf', 'biorob_panda.urdf.xacro'
    )
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file,
        ' hand:=false',
        ' ee_id:=none',
        ' robot_ip:=dont-care',
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'),
        ' ros2_control:=false'
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_config, value_type=str)
    }

    # 2. Semantic Robot Description (SRDF xacro)
    biorob_semantic_xacro_file = os.path.join(
        get_package_share_directory('phantom_panda_teleop'),
        'urdf', 'biorob_panda.srdf.xacro'
    )
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ', biorob_semantic_xacro_file,
        ' hand:=false', ' ee_id:=none'
    ])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_config,
            value_type=str
        )
    }

    # 3. Servo Parameters configuration
    servo_yaml = load_yaml('phantom_panda_teleop', 'config/servo_parameters.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    # 4. Joint Limits configuration
    joint_limits_yaml = load_yaml('phantom_panda_teleop', 'config/joint_limits.yaml')
    accumulator_yaml = os.path.join(
        get_package_share_directory('phantom_panda_teleop'),
        'config', 'trajectory_accumulator.yaml'
    )

    # Start MoveIt Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            # Intentionally do not load a pose-IK plugin into MoveIt Servo. On Humble,
            # the presence of one makes Servo solve every velocity increment through
            # approximate pose IK. The differential Jacobian fallback preserves the
            # commanded Cartesian twist much more faithfully for streaming teleoperation.
            # The differential Jacobian path is used for streamed teleoperation.
            servo_params,
            joint_limits_yaml
        ]
    )

    trajectory_accumulator_node = Node(
        package='phantom_panda_teleop',
        executable='servo_trajectory_accumulator_node',
        name='servo_trajectory_accumulator',
        output='screen',
        parameters=[
            accumulator_yaml,
            {
                'output_topic': LaunchConfiguration('controller_command_topic'),
                'controller_state_topic': LaunchConfiguration('controller_state_topic'),
            }
        ]
    )

    return LaunchDescription([
        use_fake_hardware_arg,
        controller_command_topic_arg,
        controller_state_topic_arg,
        servo_node,
        trajectory_accumulator_node
    ])
