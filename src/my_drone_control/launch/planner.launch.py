import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_drone_control')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'planner_view.rviz')

    return LaunchDescription([
        # Spustenie tvojho C++ uzla
        Node(
            package='my_drone_control',
            executable='map_planner_node',
            name='map_planner_node',
            output='screen'
        ),
        # Spustenie RViz2 s načítanou konfiguráciou
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])