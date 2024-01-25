from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, GroupAction
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

    gps_topic = DeclareLaunchArgument(
        'gps_topic',
        default_value='/lexus3/gps/duro/current_pose'
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

    driverModel_params = os.path.join(
        get_package_share_directory('lane_keep_system'),
        'config',
        'driverParams.yaml'
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
            {'gps_topic': LaunchConfiguration('gps_topic')},
            {'ego_frame': 'base_link'},
            {'visualize': LaunchConfiguration('visualize')},
            driverModel_params
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
            {'visualize': LaunchConfiguration('visualize')},
            driverModel_params
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

    ### MPC ###

    vehicle_info_param = os.path.join(
        get_package_share_directory('lane_keep_system'),
        'config',
        'vehicle_info.param.yaml'
    )

    nearest_search_param = os.path.join(
        get_package_share_directory('lane_keep_system'),
        'config',
        'nearest_search.param.yaml'
    )

    trajectory_follower_node_param = os.path.join(
        get_package_share_directory('lane_keep_system'),
        'config',
        'trajectory_follower_node.param.yaml'
    )

    lat_controller_param = os.path.join(
        get_package_share_directory('lane_keep_system'),
        'config',
        'mpc.param.yaml'
    )

    lon_controller_param = os.path.join(
        get_package_share_directory('lane_keep_system'),
        'config',
        'pid.param.yaml'
    )

    mpc = Node(
        package="trajectory_follower_node",
        executable="controller_node_exe",
        name="controller_node_exe",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/current_odometry", "/localization/kinematic_state"),
            ("~/input/current_steering", "/vehicle/status/steering_status"),
            ("~/input/current_accel", "/localization/acceleration"),
            ("~/input/current_operation_mode", "/system/operation_mode/state"),
            ("~/output/predicted_trajectory", "lateral/predicted_trajectory"),
            ("~/output/lateral_diagnostic", "lateral/diagnostic"),
            ("~/output/slope_angle", "longitudinal/slope_angle"),
            ("~/output/longitudinal_diagnostic", "longitudinal/diagnostic"),
            ("~/output/control_cmd", "/control/command/control_cmd"),
        ],
        parameters=[
            {
                "lateral_controller_mode": "mpc",
                "longitudinal_controller_mode": "pid",
            },
            nearest_search_param,
            trajectory_follower_node_param,
            lon_controller_param,
            lat_controller_param,
            vehicle_info_param,
        ],
        output="screen",
    )
    

    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace('ldm'),
                
                lanelet_frame,
                gps_topic,
                gps_yaw_offset,
                visualize,
                lanelet2_path,
                lanelet_map_handler,
                trajectory_planner,
                # mpc_steering_path,
                rviz,
                mpc
            ]
        )
    ])
