"""
f1tenth_rl.launch.py

rl_driver と hardware_bridge の2ノードを同時起動するlaunchファイル。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        default_value=os.path.join(home_dir, 'projects/f1tenth-project/models/0_magp_late.onnx'),
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

    speed_multiplier_arg = DeclareLaunchArgument(
        'speed_multiplier', default_value='1.0',
        description='Multiplier for AI predicted speed'
    )

    steer_multiplier_arg = DeclareLaunchArgument(
        'steer_multiplier', default_value='1.0',
        description='Multiplier for AI predicted steering'
    )

    racing_line_arg = DeclareLaunchArgument(
        'racing_line_path', default_value='',
        description='Path to racing line CSV (optional)'
    )

    slam_arg = DeclareLaunchArgument(
        'slam', default_value='True',
        description='Whether to automatically launch Hokuyo LiDAR and Cartographer SLAM in the background'
    )

    # ─── Nodes ───
    rl_driver_node = Node(
        package='f1tenth_rl',
        executable='rl_driver',
        name='rl_driver',
        output='screen',
        parameters=[
            config_path,
            {
                'model_path': LaunchConfiguration('model_path'),
                'racing_line_path': LaunchConfiguration('racing_line_path'),
                'speed_multiplier': LaunchConfiguration('speed_multiplier'),
                'steer_multiplier': LaunchConfiguration('steer_multiplier')
            }
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

    # ─── 静的TF (base_link -> laser) ───
    # これにより Cartographer が LiDARの物理的な搭載位置を認識できるようになります
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0.11', '0.0', '0.12', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        output='screen'
    )

    # ─── 外部Launchファイルのインクルード (LiDAR + Cartographer SLAM) ───
    # 1. LiDAR (urg_node2)
    try:
        urg_node2_launch = os.path.join(
            get_package_share_directory('urg_node2'),
            'launch', 'urg_node2.launch.py'
        )
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_node2_launch),
            condition=IfCondition(LaunchConfiguration('slam'))
        )
    except Exception:
        lidar_launch = None

    # 2. SLAM (Cartographer)
    try:
        carto_launch_file = os.path.join(
            get_package_share_directory('f1tenth_mapping'),
            'launch', 'cartographer.launch.py'
        )
        carto_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(carto_launch_file),
            condition=IfCondition(LaunchConfiguration('slam'))
        )
    except Exception:
        carto_launch = None

    # ─── LaunchDescriptionの構築と返却 ───
    ld = LaunchDescription([
        model_path_arg,
        rviz_arg,
        fixed_speed_mode_arg,
        fixed_esc_duty_arg,
        speed_multiplier_arg,
        steer_multiplier_arg,
        racing_line_arg,
        slam_arg,
        rl_driver_node,
        hardware_bridge_node,
        rviz_node,
        static_tf_node
    ])

    if lidar_launch is not None:
        ld.add_action(lidar_launch)
    if carto_launch is not None:
        ld.add_action(carto_launch)

    return ld
