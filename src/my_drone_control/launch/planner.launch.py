import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('my_drone_control')
    repo_dir = os.environ.get('LRS_REPO_DIR', os.path.expanduser('~/LRS-URK'))
    scripts_dir = os.path.join(repo_dir, 'scripts')

    if sys.stdin.isatty():
        answer = input('Open RViz? [y/N]: ').strip().lower()
        show_rviz = answer in ('y', 'yes')
    else:
        show_rviz = False
        print('No interactive terminal detected; starting without RViz.')

    planner = Node(
        package='my_drone_control',
        executable='map_planner_node',
        name='map_planner_node',
        parameters=[{
            'map_path': os.path.join(package_dir, 'maps', 'map.pcd'),
            'mission_path': os.path.join(package_dir, 'missions', 'exam_example.csv'),
        }],
        output='screen',
    )

    gazebo = ExecuteProcess(
        cmd=['bash', os.path.join(scripts_dir, 'run_gazebo.sh')],
        cwd=repo_dir,
        name='gazebo',
        output='screen',
    )
    sitl = ExecuteProcess(
        cmd=['bash', os.path.join(scripts_dir, 'run_sitl.sh')],
        cwd=repo_dir,
        name='ardupilot_sitl',
        output='screen',
    )
    mavros = ExecuteProcess(
        cmd=['bash', os.path.join(scripts_dir, 'run_mavros.sh')],
        cwd=repo_dir,
        name='mavros',
        output='screen',
    )

    next_after_planner = []
    if show_rviz:
        next_after_planner.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_dir, 'config', 'planner_view.rviz')],
            output='screen',
        ))
    next_after_planner.append(gazebo)

    return LaunchDescription([
        RegisterEventHandler(
            OnProcessStart(
                target_action=planner,
                on_start=next_after_planner,
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=gazebo,
                on_start=[TimerAction(period=5.0, actions=[sitl])],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=sitl,
                on_start=[TimerAction(period=5.0, actions=[mavros])],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=mavros,
                on_start=[TimerAction(
                    period=2.0,
                    actions=[Node(
                        package='my_drone_control',
                        executable='takeoff_drone',
                        name='takeoff_drone',
                        output='screen',
                    )],
                )],
            )
        ),
        planner,
    ])