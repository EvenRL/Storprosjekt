from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller_pkg',  # name of  package
            executable='moveit_controller',  #  name of the executable
            name='moveit_controller_node',  #name of the node
            output='screen'
        ),
        Node(
            package='robot_controller_pkg',
            executable='coordinator_node',
            name='coordinator_node',
            output='screen'
        )
    ])
