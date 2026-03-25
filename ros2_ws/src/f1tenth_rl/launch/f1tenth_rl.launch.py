"""
f1tenth_rl.launch.py

rl_driver と hardware_bridge の2ノードを同時起動するlaunchファイル。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # ─── 設定ファイル (YAML) のパス取得 ───
    from ament_index_python.packages import get_package_share_directory
    try:
        # インストール済みのパッケージから取得
        config_path = os.path.join(get_package_share_directory('f1tenth_rl'), 'config', 'params.yaml')
    except Exception:
        # 開発環境用フォールバック (ソースディレクトリ)
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'params.yaml'
        )

    # ─── Launch Arguments ───
    home_dir = os.path.expanduser('~')
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(home_dir, 'projects/jetson-ros2-project/ros2_ws/models/model'),
        description='Path to the trained PPO model (absolute or relative to home)'
    )

    # ─── Nodes ───
    rl_driver_node = Node(
        package='f1tenth_rl',
        executable='rl_driver',
        name='rl_driver',
        output='screen',
        parameters=[
            config_path,
            {'model_path': LaunchConfiguration('model_path')}
        ]
    )

    hardware_bridge_node = Node(
        package='f1tenth_rl',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
        parameters=[config_path]
    )

    return LaunchDescription([
        model_path_arg,
        rl_driver_node,
        hardware_bridge_node,
    ])
