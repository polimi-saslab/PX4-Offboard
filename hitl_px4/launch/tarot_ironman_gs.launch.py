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
        executable='frame_broadcaster',
        name='frame_broadcaster_node',
        parameters=[{
            'use_sim_time': False
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
            'target_twist_topic': 'pitot_1/sensor_speed_FRD',
            'use_sim_time': False
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

    pitot_2_node = Node(
        package='pitot_logger',
        executable='pitot_ros_bridge',
        name='pitot_2_ros_bridge_node',
        parameters=[{
            'frame_id': 'pitot_2_FLU',
            'device_name': 'pitot_2',
            'use_sim_time': False
        }]
    )

    static_transform_publisher_pitot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.08', '0.0', '-0.57', '3.1415', '2.9670', '0', 'vehicle_FRD', 'pitot_1_FLU'],
        # x y z yaw pitch roll parent_frame child_frame
        name='static_transforms_publisher_vehicle_pitot_1',
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

    attitude_rc_setpoint = Node(
        package='px4_ros_bridge',
        executable='attitude_rc_setpoint',
        name='attitude_rc_setpoint'
    )

    return LaunchDescription([   
        DeclareLaunchArgument(
            'xrce_serial_port',
            default_value=xrce_serial_port,
            description='Serial port for micro XRCE-DDS agent'
        ),
        micro_XRCE_agent,
        odom_broadcaster_node,
        rigid_twist_calculator_node,
        bag_recorder_node,
        pitot_1_node,
        # pitot_2_node,
        static_transform_publisher_odom_NED,
        static_transform_publisher_pitot1,
        # static_transform_publisher_pitot2,
        px4_ros_bridge,
        attitude_rc_setpoint
    ])