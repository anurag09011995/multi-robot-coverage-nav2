from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    # Find package
    pkg_share = FindPackageShare('mobile2_bot_description')

    # Paths to existing launch files
    gazebo_launch = PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])
    spawn_one_launch = PathJoinSubstitution([pkg_share, 'launch', 'spawn_one.launch.py'])

    # Launch Gazebo world first
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'gui': 'false',
            # 'headless': 'true'
        }.items()
    )

    # Spawn first robot
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_one_launch),
        launch_arguments={
            'namespace': 'robot1',
            'prefix': 'robot1',
            'x': '0.0',
            'y': '0.0',
            'z': '0.01',
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn second robot (offset position)
    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_one_launch),
        launch_arguments={
            'namespace': 'robot2',
            'prefix': 'robot2',
            'x': '1.5',
            'y': '1.0',
            'z': '0.01',
            'use_sim_time': use_sim_time
        }.items()
    )

    # Log info for debugging
    info_msg = LogInfo(msg="Launching Gazebo with two robots: robot1 and robot2")

    # Delay robot2 spawn slightly so Gazebo has time to register robot1
    # delayed_robot1 = TimerAction(period=3.0, actions=[robot1])

    delayed_robot2 = TimerAction(period=6.0, actions=[robot2])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description= 'Use simulation (Gazebo) clock if true'
        ),
        info_msg,
        gazebo,
        robot1,
        delayed_robot2
    ])
