from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os

def generate_launch_description():

    micro_XRCE_agent = ExecuteProcess(
        cmd=[
            FindExecutable(name='MicroXRCEAgent'),
            'serial',
            '--dev',
            '/dev/' + 'pix_UART_1',
            '-b',
            '921600'
        ],
        shell=False,
        output='screen',
    )

    odom_broadcaster_node = Node(
        package='px4_custom_fn',
        executable='odom_broadcaster',
        name='odom_broadcaster_node'
    )

    pitot_1_logger_node = Node(
        package='pitot_logger',
        executable='pitot_ros_bridge',
        name='pitot_ros_bridge_node',
        parameters=[{
            'frame_id': 'vehicle',
            'device_name': 'pitot_1'
        }],
        # execute only if device pitot_1 is present
        # condition=IfCondition(
        #     PythonExpression([
        #         'os.path.exists("/dev/pitot_1")'
        #     ])
        # )
    )

    pitot_2_logger_node = Node(
        package='pitot_logger',
        executable='pitot_ros_bridge',
        name='pitot_ros_bridge_node',
        parameters=[{
            'frame_id': 'vehicle',
            'device_name': 'pitot_2'
        }],
        # execute only if device pitot_2 is present
        # condition=IfCondition(
        #     PythonExpression([
        #         'os.path.exists("/dev/pitot_2")'
        #     ])
        # )
    )

    return LaunchDescription(
        [
            micro_XRCE_agent,
            odom_broadcaster_node,
            pitot_1_logger_node,
            pitot_2_logger_node,
        ]
    )