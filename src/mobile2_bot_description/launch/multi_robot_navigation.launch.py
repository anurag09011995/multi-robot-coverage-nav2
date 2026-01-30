from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter
from launch_ros.actions import SetRemap

def generate_launch_description():

    sim_time_param = SetParameter(name='use_sim_time', value='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart   = LaunchConfiguration('autostart', default='true')
    map_yaml    = '/home/anuragpa/maps/global_map.yaml'

    # Robot-specific parameter files
    robot1_params = PathJoinSubstitution([
        FindPackageShare('mobile2_bot_description'),
        'config',
        'nav2_robot1.yaml'
    ])

    robot2_params = PathJoinSubstitution([
        FindPackageShare('mobile2_bot_description'),
        'config',
        'nav2_robot2.yaml'
    ])

    # Nav2 bringup
    nav2_bringup = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "bringup_launch.py"
    ])


    # ─────────────────────────────────────────────
    # Robot 1
    # ─────────────────────────────────────────────
    robot1_nav = GroupAction([

        LogInfo(msg="=== Launching Nav2 for Robot1 ==="),

        SetRemap(src='tf', dst='/tf'),
        SetRemap(src='tf_static', dst='/tf_static'),
        SetRemap(src='/tf', dst='/tf'),
        SetRemap(src='/tf_static', dst='/tf_static'),

        SetRemap(src='map', dst='/map'),

        # 3. FORCE SIM TIME (Double Check)
        SetParameter(name='use_sim_time', value='true'),

        # FORCE STATIC TRANSFORM (Map -> Robot1Odom)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='robot1_map_to_odom_static',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot1odom'],
        #     parameters=[{'use_sim_time': True}]
        # ),

        

        # NAV2 Bringup inside robot1 namespace
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup),
            launch_arguments={
                'use_sim_time':  use_sim_time,
                'autostart':     autostart,
                'params_file':   robot1_params,
                'namespace':     'robot1',
                'use_namespace': 'true',
                'map': map_yaml,
                'use_composition': 'False'
            }.items()
        )
    ])


    # Robot 2

    robot2_nav = GroupAction([

        LogInfo(msg="=== Launching Nav2 for Robot2 ==="),

        # 1. FORCE GLOBAL TOPICS
        SetRemap(src='tf', dst='/tf'),
        SetRemap(src='tf_static', dst='/tf_static'),
        SetRemap(src='/tf', dst='/tf'),
        SetRemap(src='/tf_static', dst='/tf_static'),
        
        # 2. FORCE GLOBAL MAP
        SetRemap(src='map', dst='/map'),

        # 3. FORCE SIM TIME
        SetParameter(name='use_sim_time', value='true'),

        # FORCE STATIC TRANSFORM (Map -> Robot1Odom)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='robot2_map_to_odom_static',
        #     arguments=['5.0', '2.5', '0', '0', '0', '0', 'map', 'robot2odom'],
        #     parameters=[{'use_sim_time': True}]
        # ),

        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup),
            launch_arguments={
                'use_sim_time':  use_sim_time,
                'autostart':     autostart,
                'params_file':   robot2_params,
                'namespace':     'robot2',
                'use_namespace': 'true',
                'map': map_yaml, 
                'use_composition': 'False'
            }.items()
        )
    ])

    # MAIN DESCRIPTION
    return LaunchDescription([
        sim_time_param,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),

        # Start after Gazebo spawns both robots
        TimerAction(period=3.0, actions=[robot1_nav, robot2_nav])
    ])
