"""
simulation.launch.py

PC上で強化学習モデルまたはPure Pursuitコントローラの自動追従走行を擬似的に検証（SILシミュレーション）するためのLaunchファイル。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_rl = get_package_share_directory('f1tenth_rl')
    pkg_share_map = get_package_share_directory('f1tenth_mapping')
    
    # ─── 設定ファイル (params.yaml) ───
    config_path = os.path.join(pkg_share_rl, 'config', 'params.yaml')

    # ─── 起動引数 (Launch Arguments) ───
    home_dir = os.path.expanduser('~')
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(home_dir, 'projects/f1tenth-project/models/0_magp_late.onnx'),
        description='Path to the trained PPO model'
    )

    racing_line_arg = DeclareLaunchArgument(
        'racing_line_path',
        default_value=os.path.join(home_dir, 'projects/f1tenth-project/maps/map_1_0509_145516_centerline.csv'),
        description='Path to racing line CSV'
    )

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to the map YAML file (optional) to display as background in RViz'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Launch RViz2 for 3D simulation visualization'
    )

    # ─── 0. 地図パブリッシャーノード (Map Server & Lifecycle Manager) ───
    # map 引数が空文字でない場合のみ自動で起動するよう条件設定
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map'), "' != ''"]))
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']}
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map'), "' != ''"]))
    )

    # ─── 1. 模擬ロボット (Mock Robot - TF & Scan & Odom Simulator) ───
    mock_robot_node = Node(
        package='f1tenth_mapping',
        executable='mock_robot',
        name='mock_robot',
        output='screen',
        parameters=[
            {'racing_line_path': LaunchConfiguration('racing_line_path')}
        ]
    )

    # ─── 2. 静的座標変換 (base_link -> laser) ───
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_static_tf_publisher',
        arguments=['0.11', '0.0', '0.12', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        output='screen'
    )

    # ─── 3. 自律走行ノード (RL Driver) ───
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
                'use_sim_to_real': False  # シミュレータ上なのでノイズ追加などはオフ
            }
        ]
    )

    # ─── 4. 視覚化ツール (RViz2) ───
    rviz_config_dir = os.path.join(pkg_share_rl, 'rviz', 'f1tenth_rl.rviz')
    # もし rvizファイルがなければ f1tenth_mapping のものを使用
    if not os.path.exists(rviz_config_dir):
        rviz_config_dir = os.path.join(pkg_share_map, 'rviz', 'mapping.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_sim',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        racing_line_arg,
        map_arg,
        rviz_arg,
        map_server_node,
        lifecycle_manager_node,
        mock_robot_node,
        static_tf_node,
        rl_driver_node,
        # 少し時間をあけてからRVizを起動し、TFツリーが構築された状態で表示する
        TimerAction(
            period=1.5,
            actions=[rviz_node]
        )
    ])
