import rclpy
from rclpy.node import Node
from sheep_simulation_interfaces.msg import EntityPose, EntityPoseArray
from sheep_simulation_interfaces.srv import EntitySpawn, Grid
import math


class WolfSimulationNode(Node):
    def __init__(self):
        super().__init__('wolf_simulation_node')

        # Timer for wolf logic
        self.timer = self.create_timer(0.1, self.update_simulation)

        # List of all wolves
        self.wolves = []

        # Services
        self.wolf_spawn_service = self.create_service(EntitySpawn, "sheep_simulation/wolf/spawn", self.wolf_spawn_callback)

        # Clients
        self.grid_init_client = self.create_client(Grid, 'sheep_simulation/grid')
        while not self.grid_init_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for grid service...')

        # Publishers
        self.wolf_position_publisher = self.create_publisher(EntityPoseArray, 'sheep_simulation/wolf/pose', 10)

        # Subscribers
        self.sheep_position_subscription = self.create_subscription(EntityPoseArray, 'sheep_simulation/sheep/pose', self.sheep_position_callback, 10)
        # self.wolf_position_subscription = self.create_subscription(Grid, 'sheep_simulation/grid', self.grid_initialisation_callback, 10)

        # Tracking sheep positions and groups
        self.sheep_positions = {}
        self.group_assignments = {}  # Maps sheep to their assigned group

        self.init_grid()


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


    def wolf_spawn_callback(self, request, response):
        try:
            for wolf in request.spawn_entities:

                wolf_obj = {
                    "name": wolf.name,
                    "pose": {
                        "x": wolf.x,
                        "y": wolf.y,
                        "theta": wolf.theta
                    }
                }

                self.wolves.append(wolf_obj)
                response.result = "ok"
                self.get_logger().info(f"Spawned sheep: {wolf.name} at ({wolf.x}, {wolf.y})")
        except Exception as e:
            response.result = "fail"
            self.get_logger().error(f"Failed to spawn wolf: {e}")
        return response

    def sheep_position_callback(self, msg):
        # Track sheep positions by their names
        for sheep in msg.entity_positions:
            self.sheep_positions[sheep.name] = (sheep.x, sheep.y)

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
        
        positions = []
        for wolf_index, wolf in enumerate(self.wolves):
            # Update the wolf's position
            if self.sheep_safe():
                self.return_to_pen(wolf["pose"], wolf_index)
            else:
                wolf["pose"] = self.herd_sheep(wolf["pose"], wolf_index)

            entity = EntityPose()
            entity.name = wolf["name"]
            entity.x = wolf["pose"]["x"]
            entity.y = wolf["pose"]["y"]
            entity.theta = wolf["pose"]["theta"]
            positions.append(entity)


        self.publish_wolf_positions(positions)
    
    def sheep_safe(self):
        return all([(pose[0] >= self.pen_x_min) and (pose[1] >= self.pen_y_min) for pose in self.sheep_positions.values()])

    def return_to_pen(self, wolf_pose, wolf_index):
        # All sheep are in the pen; return to the wolf pen if not in already
        wolf_pen_x, wolf_pen_y = self.wolf_pen_locations[f"wolf{wolf_index + 1}"]

        if not((wolf_pose["x"] <= wolf_pen_x+self.pen_size/4) and (wolf_pose["y"] <= wolf_pen_y+self.pen_size/4)):
            direction_x = wolf_pen_x - wolf_pose["x"]
            direction_y = wolf_pen_y - wolf_pose["y"]
            move_length = math.hypot(direction_x, direction_y)
            if move_length > 0:
                wolf_pose["x"] += (direction_x / move_length) * 0.5
                wolf_pose["y"] += (direction_y / move_length) * 0.5

    def herd_sheep(self, wolf_pose, wolf_index):
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


        # limit to grid walls
        wolf_pose["x"] = max(self.grid[0][0], min(wolf_pose["x"], self.grid[0][1]))
        wolf_pose["y"] = max(self.grid[1][0], min(wolf_pose["y"], self.grid[1][1]))

        return wolf_pose

    def publish_wolf_positions(self, positions):
        msg = EntityPoseArray()
        msg.entity_positions = positions
        self.wolf_position_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WolfSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
