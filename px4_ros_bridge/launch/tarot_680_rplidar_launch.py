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

    xrce_serial_port = LaunchConfiguration('xrce_serial_port', default='/dev/pix_UART_1')

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

    odom_broadcaster_node = Node(
        package='px4_ros_bridge',
        executable='odom_broadcaster',
        name='odom_broadcaster_node'
    )

    bag_recorder_node = Node(
        package='px4_ros_bridge',
        executable='telemetry_bag_recorder',
        name='telemetry_bag_recorder_node'
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
    
    static_transforms_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.05', '0.045', '3.1415', '0', '0', 'vehicle', 'laser'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_vehicle_laser',
        output='screen'
    )

    return LaunchDescription([   
        DeclareLaunchArgument(
            'xrce_serial_port',
            default_value=xrce_serial_port,
            description='Serial port for micro XRCE-DDS agent'
        ),

        micro_XRCE_agent,
        odom_broadcaster_node,
        static_transforms_publisher,
        rplidar_launch,
        bag_recorder_node
    ])
