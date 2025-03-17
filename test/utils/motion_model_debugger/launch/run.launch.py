from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define o caminho para o arquivo de configuração
    config_file = os.path.join(
        get_package_share_directory('motion_model_debugger'),
        'config',
        'setup.yaml'
    )

    # Define o nó com os parâmetros
    motion_model_debugger_node = Node(
        package='motion_model_debugger',
        executable='motion_model_debugger',
        name='motion_model_debugger',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        motion_model_debugger_node
    ])