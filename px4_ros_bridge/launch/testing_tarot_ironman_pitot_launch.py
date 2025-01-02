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
        name='odom_broadcaster_node',
        parameters=[{
            'use_sim_time': True
        }]
    )

    rigid_twist_calculator_node = Node(
        package='px4_ros_bridge',
        executable='rigid_twist_calculator',
        name='rigid_twist_calculator_node',
        parameters=[{
            'origin_frame': 'vehicle_FRD',
            'target_frame': 'pitot_1_FLU',
            'origin_twist_topic': 'vehicle_speed_vehicle',
            'target_twist_topic': 'pitot_1/sensor_speed',
            'use_sim_time': True
        }]
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
        name='pitot_1_ros_bridge_node',
        parameters=[{
            'frame_id': 'pitot_1_FLU',
            'device_name': 'pitot_1',
            'use_sim_time': False
        }]
    )

    pitot_1_raw2twist = Node(
        package='pitot_logger',
        executable='pitot_ros_raw2twist',
        name='pitot_1_raw2twist_node',
        parameters=[{
            'frame_id': 'pitot_1_FLU',
            'device_name': 'pitot_1',
            'use_sim_time': True
        }]
    )

    pitot_2_node = Node(
        package='pitot_logger',
        executable='pitot_ros_bridge',
        name='pitot_2_ros_bridge_node',
        parameters=[{
            'frame_id': 'pitot_2_FLU',
            'device_name': 'pitot_2'
        }]
    )

    static_transform_publisher_odom_NED = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # transform from ENU(odom) to NED
        arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'odom', 'odom_NED'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_odom_NED',
        output='screen'
    )   

    static_transform_publisher_pitot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.06', '0.03', '-0.47', '0', '3.1415', '0', 'vehicle_FRD', 'pitot_1_FLU'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_vehicle_pitot_1',
        output='screen'
    )

    static_transform_publisher_pitot1_backup = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.06', '0.03', '-0.47', '0', '3.1415', '0', 'vehicle_FRD', 'pitot_1'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_vehicle_pitot_1_backup',
        output='screen'
    )   

    static_transform_publisher_pitot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.05', '0.045', '0', '0', '0', 'vehicle_FRD', 'pitot_2_FLU'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_vehicle_pitot_2',
        output='screen'
    ) 

    px4_ros_bridge = Node(
        package='px4_ros_bridge',
        executable='px4_ros_bridge',
        name='px4_ros_bridge_node'
    )

    return LaunchDescription([   
        DeclareLaunchArgument(
            'xrce_serial_port',
            default_value=xrce_serial_port,
            description='Serial port for micro XRCE-DDS agent'
        ),
        # micro_XRCE_agent,
        odom_broadcaster_node,
        rigid_twist_calculator_node,
        # bag_recorder_node,
        # pitot_1_node,
        # pitot_2_node,
        pitot_1_raw2twist,
        static_transform_publisher_odom_NED,
        static_transform_publisher_pitot1,
        static_transform_publisher_pitot1_backup,
        # static_transform_publisher_pitot2,
        # px4_ros_bridge
    ])