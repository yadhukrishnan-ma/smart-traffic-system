"""
test_cameras.launch.py — Camera-based traffic priority test
============================================================
Launches the intersection with a controlled test spawner that places
5/4/3/2 cars per direction. The traffic controller holds ALL_RED for
30s so all vehicles queue up, then cameras analyse density and the
system should give green to the busiest road first.

Usage:
  cd ~/smart_traffic_ws && source install/setup.bash
  ros2 launch smart_intersection test_cameras.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('smart_intersection')
    world_file = os.path.join(pkg_share, 'worlds', 'intersection.world')

    model_path = os.path.join(pkg_share, 'models')
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f'{model_path}:{existing}'
    )

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items()
    )

    # 2. Traffic controller (8s delay)
    #    Uses the existing 20s startup ALL_RED — enough for test spawn
    traffic = TimerAction(
        period=8.0,
        actions=[Node(
            package='smart_intersection',
            executable='smart_traffic_controller.py',
            name='smart_traffic_controller',
            output='screen',
            parameters=[{
                'startup_red_time': 30.0,   # extended for test mode
            }],
        )]
    )

    # 3. Vehicle counter (10s delay — start before test spawner)
    counter = TimerAction(
        period=10.0,
        actions=[Node(
            package='smart_intersection',
            executable='vehicle_counter.py',
            name='vehicle_counter',
            output='screen',
        )]
    )

    # 4. Emergency detector (12s delay)
    detector = TimerAction(
        period=12.0,
        actions=[Node(
            package='smart_intersection',
            executable='emergency_detector.py',
            name='emergency_detector',
            output='screen',
        )]
    )

    # 5. TEST spawner (15s delay — spawns 5+4+3+2 = 14 cars)
    #    Replaces the normal vehicle_spawner for this test
    test_spawner = TimerAction(
        period=15.0,
        actions=[Node(
            package='smart_intersection',
            executable='test_spawner.py',
            name='test_spawner',
            output='screen',
        )]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        traffic,
        counter,
        detector,
        test_spawner,
    ])
