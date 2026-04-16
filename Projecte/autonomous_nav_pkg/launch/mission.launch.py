import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_nav = get_package_share_directory('autonomous_nav_pkg')

    # Get slam_toolbox online_async_launch.py path
    slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')
    slam_launch_path = os.path.join(slam_toolbox_pkg_dir, 'launch', 'online_async_launch.py')

    # Include SLAM Toolbox launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': 'true' # Change to false if running on real robot
        }.items()
    )

    # Mission Controller Node
    mission_controller_node = Node(
        package='autonomous_nav_pkg',
        executable='mission_controller',
        name='mission_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}] # Change to False if running on real robot
    )

    return LaunchDescription([
        slam_toolbox_launch,
        mission_controller_node
    ])
