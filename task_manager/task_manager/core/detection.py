class CubeDetectionHandler:
    def __init__(self, node, required_colors):
        self.node = node
        self.required_colors = required_colors
        self.detected_cubes = {}
        self.missing_colors = list(required_colors)
        self.cube_header = None

    def process_detection(self, msg):
        # Save the header for later use in transformations
        self.cube_header = msg.header

        # Update detected cubes
        for cube in msg.cubes:
            # Only track cubes that are in our required colors
            if cube.color in self.required_colors:
                self.detected_cubes[cube.color] = cube.pose
                self.node.get_logger().info(
                    f"Detected {cube.color} cube at position: "
                    f"({cube.pose.position.x:.2f}, {cube.pose.position.y:.2f}, {cube.pose.position.z:.2f})"
                )

        # Update missing colors
        self.missing_colors = [c for c in self.required_colors if c not in self.detected_cubes]

    def reset(self):
        self.detected_cubes = {}
        self.missing_colors = list(self.required_colors)
        self.cube_header = None

    def all_cubes_found(self):
        return len(self.missing_colors) == 0