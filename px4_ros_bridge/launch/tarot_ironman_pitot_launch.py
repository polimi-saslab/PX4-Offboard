from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration, PythonExpression
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

    odom_broadcaster_node = Node(
        package='px4_ros_bridge',
        executable='odom_broadcaster',
        name='odom_broadcaster_node'
    )

    bag_recorder_node = Node(
        package='px4_ros_bridge',
        executable='telemetry_bag_recorder',
        name='telemetry_bag_recorder_node',
        parameters=[{
            'platform_name': 'tarot_ironman',
            'test_name': 'generic_test'
        }]
    )

    pitot_1_node = Node(
        package='pitot_logger',
        executable='pitot_ros_bridge',
        name='pitot_ros_bridge_node',
        parameters=[{
            'frame_id': 'vehicle',
            'device_name': 'pitot_1'
        }]
    )

    pitot_2_node = Node(
        package='pitot_logger',
        executable='pitot_ros_bridge',
        name='pitot_ros_bridge_node',
        parameters=[{
            'frame_id': 'vehicle',
            'device_name': 'pitot_2'
        }]
    )

    static_transform_publisher_pitot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.21', '-0.03', '0.47', '3.1415', '0', '0', 'vehicle', 'pitot1'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_vehicle_pitot1',
        output='screen'
)   

    static_transform_publisher_pitot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.05', '0.045', '0', '0', '0', 'vehicle', 'pitot2'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_vehicle_pitot2',
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
        bag_recorder_node,
        pitot_1_node,
        pitot_2_node,
        static_transform_publisher_pitot1,
        static_transform_publisher_pitot2
    ])