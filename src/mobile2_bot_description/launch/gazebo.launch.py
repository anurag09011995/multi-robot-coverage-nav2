from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package share (as a substitution, safe to use in PathJoinSubstitution)
    pkg_share = FindPackageShare('mobile2_bot_description')

    # URDF (xacro) and World paths
    # urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'mobile2_bot_macro.urdf.xacro'])
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'room.sdf'])

    # Show resolved paths in the launch log (helps debugging)
    # dbg_urdf  = LogInfo(msg=['[DEBUG] URDF xacro: ', urdf_file])
    dbg_world = LogInfo(msg=['[DEBUG] World SDF: ', world_file])

    # Build the xacro command **with a proper space** between the executable and the file
    # robot_description_cmd = Command([FindExecutable(name='xacro'), ' ', urdf_file])

    # Ensure the parameter is treated as a plain string (avoids YAML parsing errors)
    # robot_description = {
    #     'robot_description': ParameterValue(robot_description_cmd, value_type=str)
    # }

    # Gazebo (gzserver + gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'use_sim_time': 'true',
            'server_required': 'true',
            # 'gui': 'true',      # uncomment to force GUI on/off
            # 'server': 'true',   # uncomment to force server on/off
        }.items()
    )

    # # Robot State Publisher
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #         robot_description
    #     ]
    # )

    # # Spawn robot a bit later so Gazebo is fully up
    # spawn_robot = TimerAction(
    #     period=5.0,
    #     actions=[
    #         Node(
    #             package='gazebo_ros',
    #             executable='spawn_entity.py',
    #             arguments=[
    #                 '-entity', 'mobile2_bot',
    #                 '-topic', 'robot_description',
    #                 '-package_to_model'
    #             ],
    #             output='screen'
    #         )
    #     ]
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        # dbg_urdf,
        dbg_world,
        gazebo,
        # robot_state_publisher,
        # spawn_robot
    ])
