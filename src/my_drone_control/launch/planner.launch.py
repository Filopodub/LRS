import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('my_drone_control')
    required_scripts = ('run_gazebo.sh', 'run_sitl.sh', 'run_mavros.sh')

    def find_repo(configured_repo, candidates):
        repo_candidates = [os.path.expanduser(configured_repo)] if configured_repo else candidates
        return next((
            candidate for candidate in repo_candidates
            if all(os.path.isfile(os.path.join(candidate, 'scripts', script))
                   for script in required_scripts)
        ), None)

    repo_1 = find_repo(
        os.environ.get('LRS_REPO_DIR'),
        [os.path.expanduser('~/LRS-URK')],
    )
    repo_2 = find_repo(
        os.environ.get('LRS_2_REPO_DIR'),
        [
            os.path.expanduser('~/Documents/github/LRS-URK'),
            os.path.expanduser('~/LRS-URK'),
        ],
    )
    print(f"Assignment 1 simulator repository: {repo_1 or 'not found'}")
    print(f"Assignment 2 simulator repository: {repo_2 or 'not found'}")

    interactive = sys.stdin.isatty()
    if interactive:
        choices = []
        if repo_1:
            choices.append('1')
        if repo_2:
            choices.append('2')
        if not choices:
            raise RuntimeError(
                'No simulator repository found. Set LRS_REPO_DIR and/or '
                'LRS_2_REPO_DIR to a checkout containing scripts/run_gazebo.sh, '
                'scripts/run_sitl.sh, and scripts/run_mavros.sh.'
            )
        while True:
            choice_text = ' / '.join(choices)
            assignment = input(f'Choose assignment ({choice_text}): ').strip()
            if assignment in choices:
                break
            print(f"Choose an available assignment: {choice_text}.")
    else:
        assignment = '1' if repo_1 else '2' if repo_2 else None
        if assignment is None:
            raise RuntimeError(
                'No simulator repository found. Set LRS_REPO_DIR and/or '
                'LRS_2_REPO_DIR to a valid LRS-URK checkout.'
            )
        print(f'No interactive terminal detected; starting assignment {assignment}.')

    repo_dir = repo_1 if assignment == '1' else repo_2
    scripts_dir = os.path.join(repo_dir, 'scripts')

    if assignment == '1' and interactive:
        answer = input('Open RViz? [y/N]: ').strip().lower()
        show_rviz = answer in ('y', 'yes')
    else:
        show_rviz = False
        if assignment == '1':
            print('Starting assignment 1 without RViz.')

    if assignment == '1':
        mission_node = Node(
            package='my_drone_control',
            executable='map_planner_node',
            name='map_planner_node',
            parameters=[{
                'map_path': os.path.join(package_dir, 'maps', 'map.pcd'),
                'mission_path': os.path.join(package_dir, 'missions', 'exam_example.csv'),
            }],
            output='screen',
        )
    else:
        mission_node = Node(
            package='my_drone_control',
            executable='outdoor_mission',
            name='outdoor_mission',
            output='screen',
        )

    gazebo = ExecuteProcess(
        cmd=['bash', os.path.join(scripts_dir, 'run_gazebo.sh')] +
            (['fei_lrs_outdoor.world'] if assignment == '2' else []),
        cwd=repo_dir,
        name='gazebo',
        output='screen',
    )
    sitl = ExecuteProcess(
        cmd=['bash', os.path.join(scripts_dir, 'run_sitl.sh')],
        cwd=repo_dir,
        name='ardupilot_sitl',
        additional_env=(
            {'HOME_LOCATION': os.environ.get(
                'HOME_LOCATION', '48.15144444,17.07375000,150,0')}
            if assignment == '2' else {}
        ),
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
                target_action=mission_node,
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
                    period=10.0,
                    actions=[Node(
                        package='my_drone_control',
                        executable='takeoff_drone',
                        name='takeoff_drone',
                        output='screen',
                    )],
                )],
            )
        ),
        mission_node,
    ])