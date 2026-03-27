"""
f1tenth_rl.launch.py

rl_driver と hardware_bridge の2ノードを同時起動するlaunchファイル。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ─── 設定ファイル (YAML) のパス取得 ───
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
        default_value=os.path.join(home_dir, 'projects/jetson-ros2-project/models/model'),
        description='Path to the trained PPO model (absolute or relative to home)'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='False',
        description='Launch RViz2 for visualization'
    )

    fixed_speed_mode_arg = DeclareLaunchArgument(
        'fixed_speed_mode', default_value='True',
        description='Use fixed speed mode instead of AI predicted speed'
    )

    fixed_esc_duty_arg = DeclareLaunchArgument(
        'fixed_esc_duty', default_value='5600',
        description='Fixed PWM duty cycle for ESC in fixed speed mode'
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
        parameters=[
            config_path,
            {
                'fixed_speed_mode': LaunchConfiguration('fixed_speed_mode'),
                'fixed_esc_duty': LaunchConfiguration('fixed_esc_duty')
            }
        ]
    )
    
    # RViz2 ノード
    rviz_config_dir = os.path.join(
        get_package_share_directory('f1tenth_rl'),
        'rviz', 'f1tenth_rl.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        rviz_arg,
        fixed_speed_mode_arg,
        fixed_esc_duty_arg,
        rl_driver_node,
        hardware_bridge_node,
        rviz_node
    ])
