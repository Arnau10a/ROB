import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Configurable argument: set to 'true' for simulation, 'false' for real robot
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (true) or real clock (false)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # SLAM Toolbox — online async SLAM
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    slam_launch_path = os.path.join(
        slam_toolbox_pkg, 'launch', 'online_async_launch.py'
    )
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Mission Controller Node
    mission_controller_node = Node(
        package='autonomous_nav_pkg',
        executable='mission_controller',
        name='mission_controller_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_launch,
        mission_controller_node,
    ])
