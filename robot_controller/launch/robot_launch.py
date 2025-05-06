from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller_pkg',  # The name of your package
            executable='moveit_controller',  # The name of the executable
            name='moveit_controller_node',  # Optional: name of the node
            output='screen'
        ),
        Node(
            package='robot_controller_pkg',
            executable='coordinator_node',
            name='coordinator_node',
            output='screen'
        )
    ])
