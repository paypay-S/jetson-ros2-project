from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    try:
        config_path = os.path.join(get_package_share_directory('f1tenth_rl'), 'config', 'params.yaml')
    except Exception:
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'params.yaml'
        )

    hardware_bridge_node = Node(
        package='f1tenth_rl',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
        parameters=[config_path]
    )

    right_wall_follow_node = Node(
        package='f1tenth_rl',
        executable='right_wall_follow',
        name='right_wall_follow',
        output='screen'
    )

    return LaunchDescription([
        hardware_bridge_node,
        right_wall_follow_node,
    ])
