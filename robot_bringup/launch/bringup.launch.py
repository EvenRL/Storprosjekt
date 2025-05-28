from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import os

def generate_launch_description():

    camera_transform_path = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'camera_transform.yaml'
    )

    with open(camera_transform_path, 'r') as f:
        cam_tf_config = yaml.safe_load(f)['static_transform']

    return LaunchDescription([
        # Publish static transform from camera to tool
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_tool_tf',
            arguments=[
                str(cam_tf_config['x']),
                str(cam_tf_config['y']),
                str(cam_tf_config['z']),
                str(cam_tf_config['roll']),
                str(cam_tf_config['pitch']),
                str(cam_tf_config['yaw']),
                cam_tf_config['parent_frame'],
                cam_tf_config['child_frame'],
            ]),
        # Kjør launch fil fra cube_detector pakke
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cube_detector'),
                    'launch',
                    'detect.launch.py'
                ])
            ])
        ),
        Node(
            package='task_manager',
            executable='task_manager_node'
        )
    ])