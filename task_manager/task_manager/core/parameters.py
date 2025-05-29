from geometry_msgs.msg import Point

class ParameterManager:
    def __init__(self, node):
        self.node = node
        self.declare_parameters()
        self.config = self.read_parameters()

    def declare_parameters(self):
        # Basic parameters
        self.node.declare_parameter('sim_mode', False)
        self.node.declare_parameter('required_colors', ['green', 'black', 'blue'])
        self.node.declare_parameter('home_position', [0.1, -1.4, 0.2, -1.2, 0.1, 0.0])
        self.node.declare_parameter('overview_position', [0.0, -1.0, 0.5, -1.0, 0.0, 0.0])
        self.node.declare_parameter('place_position', [0.3, 0.3, 0.1])
        self.node.declare_parameter('joint_names', [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ])
        self.node.declare_parameter('trajectory_action_name',
                                    '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.node.declare_parameter('output_frame', 'base_link')
        self.node.declare_parameter('num_search_positions', 2)

        # Enhanced parameters
        self.node.declare_parameter('enable_auto_start', False)
        self.node.declare_parameter('enable_visualization', True)
        self.node.declare_parameter('pointing_distance', 0.15)
        self.node.declare_parameter('movement_timeout', 30.0)
        self.node.declare_parameter('retry_attempts', 3)
        self.node.declare_parameter('recovery_behavior', 'home')  # 'home', 'continue', 'abort'

        # Declare search positions
        for i in range(10):  # Support up to 10 positions
            param_name = f'search_position_{i}'
            self.node.declare_parameter(param_name, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def read_parameters(self):
        config = {}
        config['sim_mode'] = self.node.get_parameter('sim_mode').value
        config['required_colors'] = self.node.get_parameter('required_colors').value
        config['home_position'] = self.node.get_parameter('home_position').value
        config['overview_position'] = self.node.get_parameter('overview_position').value

        place_pos = self.node.get_parameter('place_position').value
        config['place_position'] = Point(x=place_pos[0], y=place_pos[1], z=place_pos[2])

        config['joint_names'] = self.node.get_parameter('joint_names').value
        config['traj_action_name'] = self.node.get_parameter('trajectory_action_name').value
        config['output_frame'] = self.node.get_parameter('output_frame').value
        config['pointing_distance'] = self.node.get_parameter('pointing_distance').value
        config['movement_timeout'] = self.node.get_parameter('movement_timeout').value
        config['retry_attempts'] = self.node.get_parameter('retry_attempts').value
        config['recovery_behavior'] = self.node.get_parameter('recovery_behavior').value
        config['enable_auto_start'] = self.node.get_parameter('enable_auto_start').value
        config['enable_visualization'] = self.node.get_parameter('enable_visualization').value

        # Reconstruct search positions from individual parameters
        config['search_positions'] = []
        num_positions = self.node.get_parameter('num_search_positions').value
        for i in range(num_positions):
            pos = self.node.get_parameter(f'search_position_{i}').value
            if pos:
                config['search_positions'].append(pos)

        return config