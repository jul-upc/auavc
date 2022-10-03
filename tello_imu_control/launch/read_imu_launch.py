from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import ament_index_python

import os

def generate_launch_description():
    
    ld = LaunchDescription()

    config = os.path.join(
    get_package_share_directory('tello_imu_control'),
    'config',
    'read_imu_config.yaml'
    )

    imu_controller = Node(
            package = 'tello_imu_control',
            node_executable = 'read_imu',
            node_namespace = '',
            remappings = [], # -> TODO: change it by [(<node_topic_name>, <destination_topic_name>)]
            parameters= [config],
            )

    record = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', 'bags_name_with_path'],
            output='screen'
            )   

    ld.add_action(imu_controller)
#    ld.add_action(record)       

    return ld