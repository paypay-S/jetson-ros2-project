import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_mapping')
    rviz_config_dir = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    mapping_launch = os.path.join(pkg_share, 'launch', 'mapping.launch.py')

    return LaunchDescription([
        # 1. Mock Robot (Fake TF and LiDAR)
        Node(
            package='f1tenth_mapping',
            executable='mock_robot',
            name='mock_robot',
            output='screen'
        ),
        
        # 2. Start SLAM Toolbox (reuse the existing mapping.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),

        # 3. Start RViz exactly 2 seconds after so TF trees are initialized
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_dir],
                    output='screen'
                )
            ]
        )
    ])
