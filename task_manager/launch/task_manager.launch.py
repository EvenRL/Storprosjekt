from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_param = DeclareLaunchArgument(
        'use_sim', 
        default_value='false',
        description='Use simulation time')
        
    params_file_param = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('task_manager'),
            'config',
            'task_manager_params.yaml'),
        description='Path to the parameter file')

    # Get the path to the parameters file
    params_file = LaunchConfiguration('params_file')
    
    # Task manager node
    task_manager_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager',
        output='screen',
        parameters=[params_file],
        emulate_tty=True  # For colored output
    )
    
    # Define launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_param)
    ld.add_action(params_file_param)
    
    # Add nodes
    ld.add_action(task_manager_node)
    
    return ld
