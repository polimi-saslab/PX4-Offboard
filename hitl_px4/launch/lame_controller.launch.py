from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os

def generate_launch_description():
    xrce_serial_port = LaunchConfiguration('xrce_serial_port', default='/dev/pix_UART_2')

    micro_XRCE_agent = ExecuteProcess(
        cmd=[
            FindExecutable(name='MicroXRCEAgent'),
            'serial',
            '--dev',
            xrce_serial_port,
            '-b',
            '921600'
        ],
        shell=False,
        output='screen',
    )

    frame_broadcaster_node = Node(
        package='px4_ros_bridge',
        executable='frame_broadcaster',
        name='frame_broadcaster_node'
    )

    rigid_transform_odom_odomNED_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'odom', 'odom_NED'],
        name='rigid_transform_publisher_node'
    )

    bag_recorder_node = Node(
        package='px4_ros_bridge',
        executable='telemetry_bag_recorder',
        name='telemetry_bag_recorder_node',
        parameters=[{
            'platform_name': 'tarot_680',
            'test_name': 'generic_test'
        }],
    )

    rplidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch',
                    'rplidar_a2m8_launch.py'
                ])
            ]),
            launch_arguments={
                'serial_port': '/dev/RPlidar',
                'frame_id': 'laser'
            }.items()
        )

    rigid_transform_odom_odomNED_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'odom', 'odom_NED'],
        name='rigid_transform_publisher_node'
    )
    
    static_transform_laser_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.05', '0.045', '3.1415', '0', '3.1415', 'vehicle_FRD', 'laser'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_vehicle_laser',
        output='screen'
    )

    px4_ros_bridge = Node(
        package='px4_ros_bridge',
        executable='px4_ros_bridge',
        name='px4_ros_bridge_node'
    )

    config_file = os.path.join(
        get_package_share_directory('px4_ros_bridge'),
        'config',
        'px4_ros_bridge_param.yaml'
    )

    obstacle_avoidance = Node(
        package='px4_ros_bridge',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[config_file]
    )

    gbeam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gbeam_controller'),
                         'launch/gbeam_controller.launch.py')
        )
    )

    lame = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lame_controller'),
                        'launch/lame_follower.launch.py')
        )
    )

    return LaunchDescription([   
        DeclareLaunchArgument(
            'xrce_serial_port',
            default_value=xrce_serial_port,
            description='Serial port for micro XRCE-DDS agent'
        ),

        micro_XRCE_agent,
        frame_broadcaster_node,
        # static_transform_laser_publisher,
        rigid_transform_odom_odomNED_publisher_node,
        # rplidar_launch,
        bag_recorder_node,
        px4_ros_bridge,
        # obstacle_avoidance,
        # gbeam
        lame
    ])
