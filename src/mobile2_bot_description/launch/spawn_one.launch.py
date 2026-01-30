from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('mobile2_bot_description').find('mobile2_bot_description')

    # Path of URDF
    urdf_file = pkg_share + '/urdf/mobile2_bot_macro.urdf.xacro'

    # Launch Arguments
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Process the urdf with prefix
    robot_description_cmd = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file,
        ' ',
        'prefix:=',
        prefix
    ])


    robot_description = {'robot_description': ParameterValue(robot_description_cmd, value_type=str)}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            robot_description,
            {'publish_frequency': 50.0}
            # {"frame_prefix": [prefix, "/"]}
        ],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', namespace,
            '-topic', [namespace, '/robot_description'],
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ],
        output='screen'
    )

    dbg_info = LogInfo(msg=[
        'Spawning robot: ',
        namespace,
        ' at position (',
        x, ', ', y, ', ', z, ') yaw=',
        yaw
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1', description='Robot namespace'),
        DeclareLaunchArgument('prefix', default_value='robot1', description='TF prefix for the robot'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn position X'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn position Y'),
        DeclareLaunchArgument('z', default_value='0.01', description='Spawn position Z'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw angle'),

        dbg_info,
        robot_state_publisher,
        spawn_entity
    ])
