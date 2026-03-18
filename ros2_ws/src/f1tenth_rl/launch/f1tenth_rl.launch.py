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
    # モデルパスの引数（デフォルトはros2_ws/models/model）
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'f1tenth-project', 'ros2_ws', 'models', 'model'
        ),
        description='Path to the trained PPO model (without .zip extension)'
    )

    rl_driver_node = Node(
        package='f1tenth_rl',
        executable='rl_driver',
        name='rl_driver',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
        }]
    )

    hardware_bridge_node = Node(
        package='f1tenth_rl',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
        parameters=[{
            # ステアリング (ch0)
            'steer_ch':        0,
            'steer_center':    4700,
            'steer_left':      3700,
            'steer_right':     5700,
            'steer_max_angle': 0.4,    # rad

            # ESC (ch1)
            'esc_ch':      1,
            'esc_stop':    5200,
            'esc_forward': 5800,
            'esc_reverse': 4000,

            # 固定速度モード
            'fixed_speed_mode': True,
            'fixed_esc_duty':   5800,
            'speed_threshold':  0.05,

            # ESCアーム待機時間
            'esc_arm_duration': 3.0,
        }]
    )

    return LaunchDescription([
        model_path_arg,
        rl_driver_node,
        hardware_bridge_node,
    ])
