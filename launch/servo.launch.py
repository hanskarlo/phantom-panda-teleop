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

    # 1. Robot Description (URDF xacro)
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file, 
        ' hand:=true', ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'), ' ros2_control:=true'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    # 2. Semantic Robot Description (SRDF xacro)
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true'
    ])
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_config, value_type=str)}

    # 3. Kinematics configuration
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # 4. Servo Parameters configuration
    servo_yaml = load_yaml('phantom_panda_teleop', 'config/servo_parameters.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    # Start MoveIt Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_params
        ]
    )

    return LaunchDescription([
        use_fake_hardware_arg,
        servo_node
    ])
