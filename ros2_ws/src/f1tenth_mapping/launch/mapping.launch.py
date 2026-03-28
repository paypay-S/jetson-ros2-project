"""
mapping.launch.py

F1TENTH マッピングセッションで必要なノードを一括起動する Launch ファイル。
起動されるノード:
  1. slam_toolbox (async_slam_toolbox_node) - LiDARスキャンからマップを構築
  2. teleop_twist_keyboard              - キーボードで車体を手動操縦

【使い方】
  ros2 launch f1tenth_mapping mapping.launch.py

【オプション引数】
  scan_topic:=/scan           LiDAR スキャントピック名（デフォルト: /scan）
  use_sim_time:=false         シミュレーション時刻使用フラグ

【注意】
  - Rviz2 はノーディスプレイ環境のため起動しない。
  - マップの保存は別途 save_and_sync.sh を実行すること。
  - LiDARドライバは別途起動済みであることを前提とする。
    （f1tenth_rl の Launch と共用、または単体で起動）
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_mapping')
    mapper_params = os.path.join(pkg_share, 'config', 'mapper_params.yaml')

    # ─── Launch 引数 ────────────────────────────────────────────────────
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LiDAR スキャントピック名'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='シミュレーション時刻を使用するか'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ─── slam_toolbox ────────────────────────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            mapper_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', LaunchConfiguration('scan_topic'))
        ]
    )

    # ─── teleop_twist_keyboard ───────────────────────────────────────────
    # キーボード入力を /cmd_vel に配信して手動走行を可能にする
    # 注意: このノードはインタラクティブ入力が必要なため、prefix で別ウィンドウ起動を推奨するが、
    #       SSH 接続時は同端末でそのまま動作する
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e' if os.environ.get('DISPLAY') else '',
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        scan_topic_arg,
        use_sim_time_arg,
        LogInfo(msg='=== F1TENTH マッピング起動 ==='),
        LogInfo(msg='手動走行: teleop_twist_keyboard でキーボード操作'),
        LogInfo(msg='マップ保存: 起動中に別ターミナルから ./scripts/save_map.sh <マップ名> を実行'),
        slam_node,
        teleop_node,
    ])
