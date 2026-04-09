import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Operation mode: mapping or drive'
    )
    
    # --- 1. LIDAR (urg_node2) ---
    urg_node2_launch = os.path.join(
        get_package_share_directory('urg_node2'),
        'launch', 'urg_node2.launch.py'
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urg_node2_launch)
    )

    # --- 2. Bridge (Twist to Ackermann + Pseudo TF) ---
    # scripts/real_bridge.py をノードとして起動
    # (注: 本来はパッケージ化が望ましいですが、一旦現行ファイルを直接呼び出します)
    real_bridge_node = Node(
        executable='python3',
        arguments=['/home/toyonishiorin/projects/f1tenth-project/scripts/real_bridge.py'],
        name='real_bridge',
        output='screen'
    )

    # --- 3. Static TF (Moved to real_bridge.py) ---
    # real_bridge.py 側で odom->base_link と同時に発行するようにしたため、ここでは不要です

    # --- 4. Hardware Bridge (Ackermann to PWM) ---
    # params.yaml を読み込む
    config_path = os.path.join(
        get_package_share_directory('f1tenth_rl'),
        'config', 'params.yaml'
    )
    hardware_bridge_node = Node(
        package='f1tenth_rl',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
        parameters=[config_path]
    )

    # --- 5. SLAM (mapping mode) ---
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_mapping'),
                'launch', 'mapping.launch.py'
            )
        )
    )

    # --- 6. Foxglove Bridge ---
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': 8765}]
    )

    return LaunchDescription([
        mode_arg,
        LogInfo(msg="=== F1TENTH Unified Bringup Starting ==="),
        lidar_launch,
        real_bridge_node,
        hardware_bridge_node,
        mapping_launch,
        foxglove_bridge_node,
    ])
