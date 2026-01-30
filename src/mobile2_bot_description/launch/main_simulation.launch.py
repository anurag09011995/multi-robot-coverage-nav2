import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter

def generate_launch_description():
    # 1. FORCE CYCLONE DDS
    # set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    # 2. FORCE SIM TIME GLOBALLY
    set_sim_time = SetParameter(name='use_sim_time', value='true')

    # --- PATHS ---
    pkg_share = FindPackageShare('mobile2_bot_description')
    nav2_launch_path = PathJoinSubstitution([pkg_share, 'launch', 'multi_robot_navigation.launch.py'])
    
    # We reconstruct the path to Gazebo launch file manually to execute it
    # Assuming it is installed to share/mobile2_bot_description/launch/two_robots.launch.py
    gazebo_launch_cmd = [
        'ros2', 'launch', 'mobile2_bot_description', 'two_robots.launch.py'
    ]

    # --- ACTION 1: LAUNCH GAZEBO (AS A SEPARATE PROCESS) ---
    # Using ExecuteProcess prevents it from auto-closing due to launch context issues
    launch_gazebo = ExecuteProcess(
        cmd=gazebo_launch_cmd,
        output='screen'
    )

    # --- ACTION 2: LAUNCH NAV2 (DELAYED) ---
    launch_nav2 = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg=">>> GAZEBO STABILIZED. LAUNCHING NAV2..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path)
            )
        ]
    )

    return LaunchDescription([
        # set_rmw,
        set_sim_time,
        launch_gazebo,
        launch_nav2
    ])