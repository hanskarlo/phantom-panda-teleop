import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    # 1. Include Touch Driver Launch
    touch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_pkg_share, 'launch', 'device_driver.launch.py')
        ),
        launch_arguments={
            'device_name': LaunchConfiguration('device_name'),
            'rviz': 'false' # We will launch our own RViz configuration if needed
        }.items()
    )

    # 2. Teleoperation Mapping Node
    teleop_node = Node(
        package='phantom_panda_teleop',
        executable='phantom_panda_teleop_node',
        name='phantom_panda_teleop_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_rcm': LaunchConfiguration('use_rcm')}
        ]
    )

    return LaunchDescription([
        device_name_arg,
        params_file_arg,
        use_rcm_arg,
        touch_driver_launch,
        teleop_node
    ])
