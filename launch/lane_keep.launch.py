from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ARGS
    lanelet_frame = DeclareLaunchArgument(
        'lanelet_frame',
        default_value='map_zala_0'
    )

    gps_yaw_offset = DeclareLaunchArgument(
        'gps_yaw_offset',
        default_value='0.02'
    )

    visualize = DeclareLaunchArgument(
        'visualize',
        default_value='true'
    )

    lanelet2_path = DeclareLaunchArgument(
        'lanelet2_path',
        default_value=os.path.join(get_package_share_directory('lane_keep_system'), 'laneletMaps', 'mw.osm'),
        description='Path to the lanelet2 map file'
    )

    # NODES
    lanelet_map_handler = Node(
        package='lane_keep_system',
        executable='lanelet_map_handler',
        name='lanelet_map_handler',
        output='screen',
        parameters=[
            {'lanelet2_path': LaunchConfiguration('lanelet2_path')},
            {'lanelet_frame': LaunchConfiguration('lanelet_frame')},
            {'ego_frame': 'base_link'},
            {'visualize': LaunchConfiguration('visualize')}
        ]
    )

    trajectory_planner = Node(
        package='lane_keep_system',
        executable='trajectory_planner',
        name='trajectory_planner',
        output='screen',
        parameters=[
            # driverParams
            {'lanelet_frame': LaunchConfiguration('lanelet_frame')},
            {'gps_yaw_offset': LaunchConfiguration('gps_yaw_offset')},
            {'visualize': LaunchConfiguration('visualize')}
        ]
    )

    mpc_steering_path = Node(
        package='lane_keep_system',
        executable='mpc_steering_path',
        name='mpc_steering_path',
        output='screen',
        parameters=[
            {'lanelet_frame': LaunchConfiguration('lanelet_frame')}
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', PathJoinSubstitution([FindPackageShare('lane_keep_system'), 'config', 'trajectory.rviz'])
        ],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )

    # TODO: MPC
    

    return LaunchDescription([
        lanelet_frame,
        gps_yaw_offset,
        visualize,
        lanelet2_path,
        lanelet_map_handler,
        # trajectory_planner,
        # mpc_steering_path,
        # bag_player,
        rviz,
        # mpc_follower
    ])
