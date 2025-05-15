from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the parameters file
    task_manager_dir = get_package_share_directory('task_manager')
    params_file = os.path.join(task_manager_dir, 'config', 'task_manager_params.yaml')
    
    # Task manager node
    task_manager_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        task_manager_node
    ])
