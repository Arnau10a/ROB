import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Obtener la ruta al archivo yaml compilado
    config_file = os.path.join(
        get_package_share_directory('phase1_pro_pkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='phase1_pro_pkg',
            executable='phase1_pro_node',
            name='phase1_pro_node',
            output='screen',
            parameters=[config_file]
        )
    ])
