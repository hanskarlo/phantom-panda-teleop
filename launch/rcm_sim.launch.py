import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('phantom_panda_teleop')
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run without RViz'
    )
    use_haptic_driver_arg = DeclareLaunchArgument(
        'use_haptic_driver',
        default_value='false',
        description='Use the physical Geomagic Touch instead of simulated haptic input'
    )
    positioning_mode_arg = DeclareLaunchArgument(
        'positioning_mode',
        default_value='haptic_jog',
        description='Registration positioning: haptic_jog, fixed, or physical_guiding'
    )
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='BioRob Haptic Device',
        description='Paired OpenHaptics device name used by the Geomagic Touch driver'
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'biorob_moveit.launch.py')
        ),
        launch_arguments={
            'robot_ip': 'dont-care',
            'controller_manager_namespace': 'rcm_sim_control',
            'use_fake_hardware': 'true',
            'headless': LaunchConfiguration('headless'),
            'rviz_config': os.path.join(package_share, 'config', 'rcm_sim.rviz'),
        }.items()
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'servo.launch.py')
        ),
        launch_arguments={
            'use_fake_hardware': 'true',
            'controller_command_topic':
                '/rcm_sim_control/fr3_arm_controller/joint_trajectory',
            'controller_state_topic':
                '/rcm_sim_control/fr3_arm_controller/controller_state',
        }.items()
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'teleop.launch.py')
        ),
        launch_arguments={
            'device_name': LaunchConfiguration('device_name'),
            'use_haptic_driver': LaunchConfiguration('use_haptic_driver'),
            'use_rcm': 'true',
            'positioning_mode': LaunchConfiguration('positioning_mode'),
            'controller_switch_service':
                '/rcm_sim_control/controller_manager/switch_controller',
            'controller_state_topic':
                '/rcm_sim_control/fr3_arm_controller/controller_state',
        }.items()
    )

    simulated_haptic = Node(
        package='phantom_panda_teleop',
        executable='sim_haptic_node',
        name='sim_haptic',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_haptic_driver'))
    )

    return LaunchDescription([
        headless_arg,
        use_haptic_driver_arg,
        positioning_mode_arg,
        device_name_arg,
        robot,
        servo,
        teleop,
        simulated_haptic,
    ])
