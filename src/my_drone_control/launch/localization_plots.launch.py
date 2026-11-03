"""Start the Assignment 2 localization sensors, vehicle stack, and plots."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    repo_dir = LaunchConfiguration('repo_dir')
    seed = LaunchConfiguration('seed')
    run_square = LaunchConfiguration('run_square')
    world = PathJoinSubstitution([
        repo_dir, 'worlds', 'fei_lrs_gazebo_localization.world'])
    model_path = PathJoinSubstitution([repo_dir, 'models'])
    existing_resource_path = EnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', default_value='')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', '--seed', seed, world],
        cwd=repo_dir,
        additional_env={
            'GZ_SIM_RESOURCE_PATH': [model_path, ':', existing_resource_path],
        },
        name='localization_gazebo',
        output='log',
    )
    sitl = ExecuteProcess(
        cmd=['bash', PathJoinSubstitution([repo_dir, 'scripts', 'run_sitl.sh'])],
        cwd=repo_dir,
        name='localization_sitl',
        output='log',
    )
    mavros = ExecuteProcess(
        cmd=['bash', PathJoinSubstitution([repo_dir, 'scripts', 'run_mavros.sh'])],
        cwd=repo_dir,
        name='localization_mavros',
        output='screen',
    )
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('localization_sensor_sim'),
            'launch',
            'localization_sensors.launch.py',
        ])),
        launch_arguments={'seed': seed}.items(),
    )
    plotter = Node(
        package='my_drone_control',
        executable='localization_plotter.py',
        name='localization_plotter',
        output='screen',
        parameters=[{'use_sim_time': True, 'window_s': 120.0}],
    )
    square_mission = Node(
        package='my_drone_control',
        executable='localization_square_mission',
        name='localization_square_mission',
        output='screen',
        condition=IfCondition(run_square),
        parameters=[{'use_sim_time': True}],
    )

    start_sensors_and_sitl = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[
                sensor_launch,
                TimerAction(period=5.0, actions=[sitl]),
            ],
        )
    )
    start_mavros = RegisterEventHandler(
        OnProcessStart(
            target_action=sitl,
            on_start=[TimerAction(period=5.0, actions=[mavros])],
        )
    )
    start_square_after_mavros = RegisterEventHandler(
        OnProcessStart(
            target_action=mavros,
            on_start=[TimerAction(period=10.0, actions=[square_mission])],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'repo_dir',
            default_value=EnvironmentVariable(
                'LRS_REPO_DIR',
                default_value=os.path.expanduser('~/LRS-URK')),
            description='Path to the LRS-URK simulator checkout',
        ),
        DeclareLaunchArgument(
            'seed', default_value='42', description='Shared Gazebo/sensor seed'),
        DeclareLaunchArgument(
            'run_square',
            default_value='false',
            description='Arm SITL and fly the optional localization characterization square',
        ),
        plotter,
        start_sensors_and_sitl,
        start_mavros,
        start_square_after_mavros,
        gazebo,
    ])