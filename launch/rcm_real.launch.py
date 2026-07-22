import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('phantom_panda_teleop')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        description='Hostname or IP address of the physical Franka robot'
    )
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='BioRob Haptic Device',
        description='Paired OpenHaptics device name'
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run without RViz'
    )
    registration_insertion_depth_arg = DeclareLaunchArgument(
        'registration_insertion_depth',
        default_value='0.08',
        description='Shaft-marker distance from trocar to distal tool tip (m)'
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'biorob_moveit.launch.py')
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'controller_manager_namespace': '',
            'use_fake_hardware': 'false',
            'headless': LaunchConfiguration('headless'),
            'rviz_config': os.path.join(package_share, 'config', 'rcm_sim.rviz'),
        }.items()
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'servo.launch.py')
        ),
        launch_arguments={
            'use_fake_hardware': 'false',
            'controller_command_topic':
                '/fr3_arm_controller/joint_trajectory',
            'controller_state_topic':
                '/fr3_arm_controller/controller_state',
        }.items()
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'teleop.launch.py')
        ),
        launch_arguments={
            'device_name': LaunchConfiguration('device_name'),
            'use_haptic_driver': 'true',
            'use_rcm': 'true',
            'positioning_mode': 'fixed',
            'start_in_registration': 'true',
            'auto_register_rcm': 'false',
            'registration_insertion_depth':
                LaunchConfiguration('registration_insertion_depth'),
            'controller_switch_service':
                '/controller_manager/switch_controller',
            'arm_controller_name': 'fr3_arm_controller',
            'controller_state_topic':
                '/fr3_arm_controller/controller_state',
        }.items()
    )

    return LaunchDescription([
        robot_ip_arg,
        device_name_arg,
        headless_arg,
        registration_insertion_depth_arg,
        robot,
        servo,
        teleop,
    ])
