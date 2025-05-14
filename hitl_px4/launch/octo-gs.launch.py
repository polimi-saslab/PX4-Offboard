from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os

def generate_launch_description():


    px4_ros_bridge_minimal = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory('px4_ros_bridge'),
            'launch',
            'px4_ros_bridge_minimal.launch.py'
        ),
        launch_arguments={'xrce_serial_port': '/dev/pix_UART_2'}.items()
    )



    return LaunchDescription([   
        px4_ros_bridge_minimal
    ])