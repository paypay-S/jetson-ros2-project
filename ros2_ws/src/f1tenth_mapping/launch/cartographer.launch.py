import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_mapping')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    cartographer_config_basename = 'f1tenth_cartographer.lua'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        respawn=True,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename
        ],
        remappings=[
            ('/echoes', '/scan'),
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        respawn=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05},
            {'publish_period_sec': 1.0}
        ]
    )
    
    # ─── rosbridge_server (Optional) ─────────────────────────────────────
    rosbridge_node = LogInfo(msg='[INFO] rosbridge_serverチェック中...')
    rosbridge_info = LogInfo(msg=' ')
    try:
        get_package_share_directory('rosbridge_server')
        from launch.actions import IncludeLaunchDescription
        from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
        rosbridge_launch = os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch',
            'rosbridge_websocket_launch.xml'
        )
        rosbridge_node = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch),
            launch_arguments={'port': '9090'}.items()
        )
        rosbridge_info = LogInfo(msg='可視化: https://studio.foxglove.dev (ws://<Jetson-IP>:9090) にて')
    except Exception as e:
        rosbridge_node = LogInfo(msg=f'[INFO] rosbridge_server スキップ: {str(e)}')

    return LaunchDescription([
        LogInfo(msg='=== F1TENTH Cartographer Mapping ==='),
        LogInfo(msg='手動走行: teleop_twist_keyboard で操作してください'),
        rosbridge_info,
        cartographer_node,
        occupancy_grid_node,
        rosbridge_node
    ])
