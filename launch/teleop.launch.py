import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths to packages
    teleop_pkg_share = get_package_share_directory('phantom_panda_teleop')
    driver_pkg_share = get_package_share_directory('geomagic_touch_x')

    # Arguments
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='BioRob Haptic Device',
        description='Name of the 3D Systems Touch haptic device'
    )

    # Path to teleop params file
    default_params_file = os.path.join(teleop_pkg_share, 'config', 'teleop_params.yaml')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the teleoperation parameter file'
    )

    use_rcm_arg = DeclareLaunchArgument(
        'use_rcm',
        default_value='true',
        description='Whether to enable Remote Center of Motion (RCM) trocar constraints'
    )
    use_haptic_driver_arg = DeclareLaunchArgument(
        'use_haptic_driver',
        default_value='true',
        description='Start the physical Geomagic Touch driver'
    )
    controller_state_topic_arg = DeclareLaunchArgument(
        'controller_state_topic',
        default_value='/fr3_arm_controller/controller_state',
        description='State feedback topic of the active arm controller'
    )
    positioning_mode_arg = DeclareLaunchArgument(
        'positioning_mode',
        default_value='fixed',
        description='Registration positioning: fixed, haptic_jog, or physical_guiding'
    )
    controller_switch_service_arg = DeclareLaunchArgument(
        'controller_switch_service',
        default_value='/controller_manager/switch_controller',
        description='ros2_control service used to release/restore the arm controller'
    )
    arm_controller_name_arg = DeclareLaunchArgument(
        'arm_controller_name',
        default_value='fr3_arm_controller',
        description='Arm command controller managed during physical guiding'
    )

    # 1. Include Touch Driver Launch
    touch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_pkg_share, 'launch', 'device_driver.launch.py')
        ),
        launch_arguments={
            'device_name': LaunchConfiguration('device_name'),
            'rviz': 'false'  # A shared RViz instance is launched separately.
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_haptic_driver'))
    )

    # 2. Teleoperation Mapping Node
    teleop_node = Node(
        package='phantom_panda_teleop',
        executable='phantom_panda_teleop_node',
        name='phantom_panda_teleop_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_rcm': LaunchConfiguration('use_rcm'),
                'controller_state_topic': LaunchConfiguration('controller_state_topic'),
                'positioning_mode': LaunchConfiguration('positioning_mode'),
                'controller_switch_service':
                    LaunchConfiguration('controller_switch_service'),
                'arm_controller_name': LaunchConfiguration('arm_controller_name'),
            }
        ]
    )

    return LaunchDescription([
        device_name_arg,
        params_file_arg,
        use_rcm_arg,
        use_haptic_driver_arg,
        controller_state_topic_arg,
        positioning_mode_arg,
        controller_switch_service_arg,
        arm_controller_name_arg,
        touch_driver_launch,
        teleop_node
    ])
