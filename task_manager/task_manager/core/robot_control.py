#from typing import List, Optional
from std_msgs.msg import String


class RobotMovementManager:
    def __init__(self, node, traj_action_name, joint_names, sim_mode=False):
        self.node = node
        self.sim_mode = sim_mode
        self.joint_names = joint_names
        self.robot_cmd_pub = node.create_publisher(String, '/robot_command', 10)

    def send_command(self, cmd, completion_callback=None):
        if self.sim_mode:
            self.node.get_logger().info(f"[SIM] Robot command: {cmd}")
            if completion_callback:
                # In simulation, simulate movement completion after a delay
                self.node.create_one_shot_timer(2.0, completion_callback)
        else:
            # Send actual command
            msg = String()
            msg.data = cmd
            self.robot_cmd_pub.publish(msg)
            self.node.get_logger().info(f"Sent robot command: {cmd}")

    def move_joints(self, position, completion_callback=None):
        position_str = ','.join(map(str, position))
        self.send_command(f"move_joints:{position_str}", completion_callback)

    def point_to(self, x, y, z, distance, completion_callback=None):
        self.send_command(f"point_to:{x},{y},{z},{distance}", completion_callback)

    def stop(self):
        self.send_command("stop")

    def format_position(self, position):
        return ','.join(map(str, position))