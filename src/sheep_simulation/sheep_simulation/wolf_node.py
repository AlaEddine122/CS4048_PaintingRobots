import rclpy
from rclpy.node import Node
from sheep_simulation_interfaces.msg import EntityPose
from sheep_simulation_interfaces.srv import EntitySpawn, Grid
import math


class WolfSimulationNode(Node):
    def __init__(self):
        super().__init__('wolf_simulation_node')

        # Timer for wolf logic
        self.timer = self.create_timer(0.5, self.update_simulation)

        # List of all wolves
        self.wolves = []

        # Services
        self.wolf_spawn_service = self.create_service(EntitySpawn, "sheep_simulation/wolf/spawn", self.wolf_spawn_callback)

        # Clients
        self.grid_init_client = self.create_client(Grid, 'sheep_simulation/grid')
        while not self.grid_init_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for grid service...')

        # Publishers
        self.wolf_position_publisher = self.create_publisher(EntityPose, 'sheep_simulation/wolf/pose', 10)

        # Subscribers
        self.sheep_position_subscription = self.create_subscription(EntityPose, 'sheep_simulation/sheep/pose', self.sheep_position_callback, 10)
        # self.wolf_position_subscription = self.create_subscription(Grid, 'sheep_simulation/grid', self.grid_initialisation_callback, 10)

        # Tracking sheep positions and groups
        self.sheep_positions = {}
        self.group_assignments = {}  # Maps sheep to their assigned group

        self.init_grid()

        # # Pen location for sheep and wolf (from master_node.py)
        # self.pen_x_min = 25.0 - 10.0
        # self.pen_y_min = 25.0 - 10.0
        # self.pen_center_x = self.pen_x_min + 5.0
        # self.pen_center_y = self.pen_y_min + 5.0

        # self.wolf_pen_x_min = -25.0
        # self.wolf_pen_y_min = 25.0 - 5.0
        # self.wolf_pen_center_x = self.wolf_pen_x_min + 5.0
        # self.wolf_pen_center_y = self.wolf_pen_y_min + 2.5

    def init_grid(self):
        # Create request
        grid_request = Grid.Request()

        future = self.grid_init_client.call_async(grid_request)

        rclpy.spin_until_future_complete(self, future)

        self.grid = [
            [future.result().xmin, future.result().xmax],
            [future.result().ymin, future.result().ymax]
        ]

        self.pen_size = future.result().pensize

        self.pen_x_min = self.grid[0][1] - self.pen_size
        self.pen_y_min = self.grid[1][1] - self.pen_size
        self.pen_x_max = self.grid[0][1]
        self.pen_y_max = self.grid[1][1]
        self.pen_center_x = self.pen_x_min + self.pen_size/2
        self.pen_center_y = self.pen_y_min + self.pen_size/2

        # Define wolf pens
        self.wolf_pen_locations = {
            "wolf1": (self.grid[0][0] + self.pen_size/4, self.grid[0][1] - self.pen_size/4),
            "wolf2": (self.grid[0][0] + self.pen_size/4, self.grid[1][0] + self.pen_size/4),
        }
        
        # self.wolf_pen_x_min = self.grid[0][0]
        # self.wolf_pen_y_min = self.grid[1][1] - pen_size/2
        # self.wolf_pen_center_x = self.wolf_pen_x_min + pen_size/4
        # self.wolf_pen_center_y = self.wolf_pen_y_min + pen_size/4


    def wolf_spawn_callback(self, request, response):
        try:
            wolf_obj = {
                "name": request.name,
                "pose": {
                    "x": request.x,
                    "y": request.y,
                    "theta": request.theta
                }
            }
            self.wolves.append(wolf_obj)
            response.result = "ok"
            self.get_logger().info(f"Spawned wolf: {request.name} at ({request.x}, {request.y})")
        except Exception as e:
            response.result = "fail"
            self.get_logger().error(f"Failed to spawn wolf: {e}")

        return response

    def sheep_position_callback(self, msg):
        # Track sheep positions by their names
        self.sheep_positions[msg.name] = (msg.x, msg.y)

    def assign_sheep_groups(self):
        """Assign sheep to groups based on proximity to wolves."""
        sheep_names = list(self.sheep_positions.keys())
        if len(self.wolves) < 2 or len(sheep_names) == 0:
            return  # Skip if there are fewer than 2 wolves or no sheep

        wolf_positions = [(wolf["pose"]["x"], wolf["pose"]["y"]) for wolf in self.wolves]

        # Assign sheep to closest wolf group
        self.group_assignments.clear()
        for sheep_name, (sheep_x, sheep_y) in self.sheep_positions.items():
            distances = [math.hypot(sheep_x - wx, sheep_y - wy) for wx, wy in wolf_positions]
            closest_wolf_index = distances.index(min(distances))
            self.group_assignments[sheep_name] = closest_wolf_index

    def update_simulation(self):
        if not hasattr(self, "grid"):
            return

        self.assign_sheep_groups()  # Update sheep group assignments
        
        for wolf_index, wolf in enumerate(self.wolves):
            # Update the wolf's position
            wolf["pose"] = self.update_wolf_position(wolf["pose"], wolf_index)

            # Publish the updated position
            self.publish_wolf_position(wolf)

    def update_wolf_position(self, wolf_pose, wolf_index):
        def is_in_pen(x, y, pen_x_min, pen_y_min):
            return pen_x_min <= x <= pen_x_min + self.pen_size and pen_y_min <= y <= pen_y_min + self.pen_size

        # Prevent the wolf from entering the sheep pen
        if is_in_pen(wolf_pose["x"], wolf_pose["y"], self.pen_x_min, self.pen_y_min):
            # Move the wolf back to its pen
            wolf_pen_x, wolf_pen_y = self.wolf_pen_locations[f"wolf{wolf_index + 1}"]
            wolf_pose["x"], wolf_pose["y"] = wolf_pen_x, wolf_pen_y
            return wolf_pose

        # Target the assigned group of sheep
        target_sheep = [
            (name, pos) for name, pos in self.sheep_positions.items()
            if self.group_assignments.get(name) == wolf_index
        ]

        if target_sheep:
            # Find the furthest sheep in the assigned group
            target_name, (sheep_x, sheep_y) = max(
                target_sheep,
                key=lambda s: math.hypot(s[1][0] - self.pen_center_x, s[1][1] - self.pen_center_y)
            )

            # Guide the sheep group to the pen
            delta_x = self.pen_center_x - sheep_x
            delta_y = self.pen_center_y - sheep_y
            theta_to_pen = math.atan2(delta_y, delta_x)

            # Move the wolf behind the furthest sheep
            distance_behind = 5.0
            behind_x = sheep_x - distance_behind * math.cos(theta_to_pen)
            behind_y = sheep_y - distance_behind * math.sin(theta_to_pen)

            # Move the wolf towards the target position
            direction_x = behind_x - wolf_pose["x"]
            direction_y = behind_y - wolf_pose["y"]
            move_length = math.hypot(direction_x, direction_y)
            if move_length > 0:
                wolf_pose["x"] += (direction_x / move_length) * 0.5
                wolf_pose["y"] += (direction_y / move_length) * 0.5
        else:
            # All sheep are in the pen; return to the wolf pen
            wolf_pen_x, wolf_pen_y = self.wolf_pen_locations[f"wolf{wolf_index + 1}"]
            direction_x = wolf_pen_x - wolf_pose["x"]
            direction_y = wolf_pen_y - wolf_pose["y"]
            move_length = math.hypot(direction_x, direction_y)
            if move_length > 0:
                wolf_pose["x"] += (direction_x / move_length) * 0.5
                wolf_pose["y"] += (direction_y / move_length) * 0.5

        # limit to grid walls
        wolf_pose["x"] = max(self.grid[0][0], min(wolf_pose["x"], self.grid[0][1]))
        wolf_pose["y"] = max(self.grid[1][0], min(wolf_pose["y"], self.grid[1][1]))

        return wolf_pose

    def publish_wolf_position(self, wolf):
        position_msg = EntityPose()
        position_msg.name = wolf["name"]
        position_msg.x = wolf["pose"]["x"]
        position_msg.y = wolf["pose"]["y"]
        position_msg.theta = wolf["pose"]["theta"]
        self.wolf_position_publisher.publish(position_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WolfSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
