from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    world_path = '/home/vitor/ros2_ws/src/my_husky_sim/worlds/my_world.world'

    return LaunchDescription([
        # Start Gazebo with your custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # Spawn the Husky model
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'husky',
                '-database', 'clearpath_husky',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'),

        # Optional: keyboard teleop
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='xterm -e',
            output='screen')
    ])

