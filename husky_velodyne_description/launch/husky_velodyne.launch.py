from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    husky_velodyne_dir = get_package_share_directory('husky_velodyne_description')
    urdf_file = os.path.join(husky_velodyne_dir, 'urdf', 'husky_with_velodyne.xacro')

    # Process the Xacro into a robot_description parameter
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('husky_gazebo'),
                    'launch',
                    'husky_playpen.launch.py'
                )
            )
        )
    ])

