import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

NAMESPACE = 'robot'

def generate_launch_description():
    dirname, filename = os.path.split(os.path.realpath(__file__))
    params = os.path.join(dirname, 'config.yaml')
    rviz_path = os.path.join(dirname, 'visualization.rviz')

    ld = LaunchDescription()

    # VineSLAM node
    vineslam = Node(
        package='vineslam_ros',
        node_executable='slam_node',
        name='slam_node',
        parameters=[params],
        namespace=NAMESPACE,
        remappings=[
            ('odom_topic', '/odom'),
            ('scan_topic', '/scan'),
            ('gps_topic', '/fix'),
            ('gps_heading_topic', '/navrelposned'),
            ('imu_topic', '/imu/rpy'),
            ('imu_data_topic', '/imu/data'),
        ],
    )
    ld.add_action(vineslam)

    # Rviz
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path, '--ros-args', '--log-level', 'INFO'],
    )
    ld.add_action(rviz)

    return ld
