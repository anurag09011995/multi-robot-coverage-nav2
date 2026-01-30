from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('mobile2_bot_description').perform(context)
    # use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    slam_params_1 = os.path.join(pkg_share, 'config', 'robot1_slam.yaml')
    # slam_params_2 = os.path.join(pkg_share, 'config', 'robot2_slam.yaml')

    nodes = [
        LogInfo(msg="Launching SLAM for robot1 and robot2..."),

        # --- Scan Stamper ---
        # TimerAction(
        #     period=3.0,
        #     actions=[
        #         Node(
        #             package='mobile2_bot_description',
        #             executable='scan_stamper',
        #             name='scan_stamper',
        #             output='screen',
        #             respawn=True,
        #             parameters=[{'use_sim_time': True}],
        #         )
        #     ]
        # ),

        # if we are facing timestamp issue, we can use above commented scan_stamper node to publish a scan results by sub to gazebo scan results and then pub to a slam so that there is no timestamp issue

        # --- SLAM for robot1 ---
        TimerAction(
            period=6.0,
            actions=[
                GroupAction([
                    LogInfo(msg="Starting SLAM for robot1"),
                    Node(
                        package='slam_toolbox',
                        executable='sync_slam_toolbox_node',
                        name='slam_toolbox_robot1',
                        namespace='robot1',
                        output='screen',
                        parameters=[{'use_sim_time': True,  'publish_tf': True}, slam_params_1],
                        remappings=[
                            ('scan', '/robot1/scan'),
                            ('map', '/map'),
                            ('/map', '/map'),
                            #('/map', '/robot1/map'),
                            ('map_metadata', '/map_metadata'),
                            ('/map_metadata', '/map_metadata'),
                            # ('/map_metadata', '/robot1/map_metadata'),
                            ('odom', '/robot1/odom'),
                            ('/odom', '/robot1/odom')
                        ]
                    )
                ])
            ]
        ),

  # WE NEED ONLY ROBOT 1 FOR THE GLOBAL_MAP BUT IF WE NEED SLAM FOR TWO ROBOTS BELOW IS THE LOGIC WHICH CAN BE USED FOR TRIGGERING 2 ROBOTS FOR THE SLAM.
        # --- SLAM for robot2 ---
        # TimerAction(
        #     period=8.0,
        #     actions=[
        #         GroupAction([
        #             LogInfo(msg="Starting SLAM for robot2"),
        #             Node(
        #                 package='slam_toolbox',
        #                 executable='sync_slam_toolbox_node',
        #                 name='slam_toolbox_robot2',
        #                 namespace='robot2',
        #                 output='screen',
        #                 parameters=[{'use_sim_time': True, 'publish_tf': True}, slam_params_2],
        #                 remappings=[
        #                     ('scan', '/robot2/scan'),
        #                     ('map', 'map'),
        #                     ('/map', '/robot2/map'),
        #                     ('map_metadata', 'map_metadata'),
        #                     ('/map_metadata', '/robot2/map_metadata'),
        #                     ('odom', '/robot2/odom'),
        #                     ('/odom', '/robot2/odom')
        #                 ]
        #             )
        #         ])
        #     ]
        # ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_robot1map',
            arguments=['0','0','0','0','0','0','1','map','robot1map'],
            output='screen'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name = 'tf2_map_to_robot2map',
        #     arguments=['0','0','0','0','0','0','1','map','robot2map'],
        #     output='screen'
        # ),

    #MAP MERGE CAN BE USED IF WE ARE USING DIFFERENT MAPS FOR TWO DIFFERENT ROBOTS BUT WE ARE USING HERE ONE WORLD MAP SO WE DO NOT NEED MAP MERGE.
    # THIS LOGIC CAN BE USED FOR MULTIPLE ROBOTS AND MULTIPLE MAPS.

        # TimerAction(
        #     period=10.0,
        #     actions=[
        #         LogInfo(msg="Starting map_merge_node after SLAMs are intialized..."),
        #         Node(
        #             package='multirobot_map_merge',
        #             executable='map_merge',
        #             name='map_merge_node',
        #             output='screen',
        #             parameters=[{
        #                 'world_frame': 'map',
        #                 'robot_map_topics': ['/robot1/map', '/robot2/map'],
        #                 'robot_pose_topics': ['/robot1/odom', '/robot2/odom'],
        #                 'known_init_poses': False,
        #                 'merging_rate': 2.0,
        #                 'discovery_rate': 1.0,
        #                 'estimation_rate': 0.5,
        #                 'publish_tf': False,
        #                 'resolution': 0.05

        #             }]
        #         )
        #     ]
        # )
    ]
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time from Gazebo'
        ),
        OpaqueFunction(function=launch_setup)
    ])
