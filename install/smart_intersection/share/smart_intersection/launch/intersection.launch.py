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

    # Ensure Gazebo can find our custom models
    model_path = os.path.join(pkg_share, 'models')
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f'{model_path}:{existing}'
    )

    # 1. Launch Gazebo with the intersection world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    # 2. Smart traffic controller (after 8 s wait for Gazebo)
    traffic = TimerAction(
        period=8.0,
        actions=[Node(
            package='smart_intersection',
            executable='smart_traffic_controller.py',
            name='smart_traffic_controller',
            output='screen',
        )]
    )

    # 3. Emergency detector (after 12 s — needs traffic controller ready)
    detector = TimerAction(
        period=12.0,
        actions=[Node(
            package='smart_intersection',
            executable='emergency_detector.py',
            name='emergency_detector',
            output='screen',
        )]
    )

    # 4. Vehicle spawner (after 15 s — needs Gazebo services fully up)
    spawner = TimerAction(
        period=15.0,
        actions=[Node(
            package='smart_intersection',
            executable='vehicle_spawner.py',
            name='vehicle_spawner',
            output='screen',
        )]
    )

    # 5. Vehicle counter (after 17 s — needs camera topics ready)
    counter = TimerAction(
        period=17.0,
        actions=[Node(
            package='smart_intersection',
            executable='vehicle_counter.py',
            name='vehicle_counter',
            output='screen',
        )]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        traffic,
        detector,
        spawner,
        counter,
    ])
