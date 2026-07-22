#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

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
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    namespace_parameter_name = 'namespace'
    controller_manager_namespace_parameter_name = 'controller_manager_namespace'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    controller_manager_namespace = LaunchConfiguration(
        controller_manager_namespace_parameter_name)
    headless = LaunchConfiguration('headless')
    rviz_config = LaunchConfiguration('rviz_config')

    # Command-line arguments

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to run headless (without RViz)'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('franka_fr3_moveit_config'),
            'rviz', 'moveit.rviz'),
        description='RViz configuration file'
    )

    # planning_context using our custom robot description wrapper
    biorob_xacro_file = os.path.join(
        get_package_share_directory('phantom_panda_teleop'),
        'urdf', 'biorob_panda.urdf.xacro'
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', biorob_xacro_file, ' hand:=false',
         ' robot_ip:=', robot_ip, ' ee_id:=none', ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands, ' ros2_control:=true'])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    biorob_semantic_xacro_file = os.path.join(
        get_package_share_directory('phantom_panda_teleop'),
        'urdf', 'biorob_panda.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         biorob_semantic_xacro_file, ' hand:=false', ' ee_id:=none']
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    kinematics_yaml = load_yaml(
        'phantom_panda_teleop', 'config/kinematics.yaml'
    )

    joint_limits_yaml = load_yaml(
        'phantom_panda_teleop', 'config/joint_limits.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
        ],
        condition=UnlessCondition(headless),
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    # Dynamically select controller yaml based on use_fake_hardware
    controllers_file_name = PythonExpression([
        "'fr3_ros_controllers_fake.yaml' if '",
        use_fake_hardware,
        "' == 'true' else 'fr3_ros_controllers.yaml'"
    ])

    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare('phantom_panda_teleop'),
        'config',
        controllers_file_name
    ])

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=controller_manager_namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', '/franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    controller_manager_path = PathJoinSubstitution([
        controller_manager_namespace,
        'controller_manager',
    ])

    # A single spawner configures both controllers sequentially and activates them
    # together. This avoids concurrent load/configure requests during startup.
    load_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster', 'fr3_arm_controller',
            '--activate-as-group',
            '--controller-manager-timeout', '60',
            '--controller-manager', controller_manager_path,
        ],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'source_list': ['franka/joint_states'], 'rate': 100}],
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'franka_robot_state_broadcaster',
            '--controller-manager', controller_manager_path,
        ],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        description='Hostname or IP address of the robot.')

    namespace_arg = DeclareLaunchArgument(
        namespace_parameter_name,
        default_value='',
        description='Namespace for the robot.'
    )
    controller_manager_namespace_arg = DeclareLaunchArgument(
        controller_manager_namespace_parameter_name,
        default_value=namespace,
        description='Namespace used to isolate the ros2_control controller manager.'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    return LaunchDescription(
        [robot_arg,
         namespace_arg,
         controller_manager_namespace_arg,
         use_fake_hardware_arg,
         fake_sensor_commands_arg,
         db_arg,
         headless_arg,
         rviz_config_arg,
         rviz_node,
         robot_state_publisher,
         ros2_control_node,
         joint_state_publisher,
         franka_robot_state_broadcaster,
         load_controllers,
         ]
    )
