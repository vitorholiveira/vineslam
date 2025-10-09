from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/root/ros2_ws/src/worlds/my_world.world', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
    ])