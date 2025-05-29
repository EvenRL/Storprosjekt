from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Load node parameters
    # Node parameters can be overriden with ros argument: <node_name>.<parameter_name>:=<value>
    detect_config = os.path.join(
      get_package_share_directory('cube_detector'),
      'config',
      'params.yaml'
      )
    
    return LaunchDescription([
        Node(
            package='cube_detector',
            executable='cube_detector',
            name='cube_detector',
            parameters=[detect_config]
        )
    ])