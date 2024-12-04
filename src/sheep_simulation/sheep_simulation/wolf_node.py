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

        # Tracking sheep positions
        self.sheep_positions = {}

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

        pen_size = future.result().pensize

        self.pen_x_min = self.grid[0][1] - pen_size
        self.pen_y_min = self.grid[1][1] - pen_size
        self.pen_x_max = self.grid[0][1]
        self.pen_y_max = self.grid[1][1]
        self.pen_center_x = self.pen_x_min + pen_size/2
        self.pen_center_y = self.pen_y_min + pen_size/2

        self.wolf_pen_x_min = self.grid[0][0]
        self.wolf_pen_y_min = self.grid[1][1] - pen_size/2
        self.wolf_pen_center_x = self.wolf_pen_x_min + pen_size/4
        self.wolf_pen_center_y = self.wolf_pen_y_min + pen_size/4


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

    def update_simulation(self):
        if not hasattr(self, "grid"):
            return
        
        for wolf in self.wolves:
            # Update the wolf's position
            wolf["pose"] = self.update_wolf_position(wolf["pose"])

            # Publish the updated position
            self.publish_wolf_position(wolf)

    def update_wolf_position(self, wolf_pose):
        def is_in_pen(x, y, pen_x_min, pen_y_min, pen_size_x=10.0, pen_size_y=10.0):
            """Check if a position is inside a specified pen."""
            return pen_x_min <= x <= pen_x_min + pen_size_x and pen_y_min <= y <= pen_y_min + pen_size_y

        # Prevent the wolf from entering the sheep pen
        if is_in_pen(wolf_pose["x"], wolf_pose["y"], self.pen_x_min, self.pen_y_min):
            # Move the wolf out of the sheep pen if inside
            wolf_pose["x"] = self.wolf_pen_center_x
            wolf_pose["y"] = self.wolf_pen_center_y
            return wolf_pose  # Skip further updates to avoid re-entering the sheep pen

        if self.sheep_positions:
            # Keep track of sheep positions and ensure they stay in the pen
            for sheep_name, (sheep_x, sheep_y) in self.sheep_positions.items():
                if is_in_pen(sheep_x, sheep_y, self.pen_x_min, self.pen_y_min):
                    # Prevent sheep from leaving the pen
                    self.sheep_positions[sheep_name] = (
                        max(self.pen_x_min, min(sheep_x, self.pen_x_min + 10.0)),
                        max(self.pen_y_min, min(sheep_y, self.pen_y_min + 10.0)),
                    )

            # Find the furthest sheep outside the pen
            furthest_sheep = None
            max_distance = -float('inf')
            for sheep_name, (sheep_x, sheep_y) in self.sheep_positions.items():
                if not is_in_pen(sheep_x, sheep_y, self.pen_x_min, self.pen_y_min):  # Only consider sheep outside the pen
                    distance_to_pen = math.hypot(sheep_x - self.pen_center_x, sheep_y - self.pen_center_y)
                    if distance_to_pen > max_distance:
                        furthest_sheep = (sheep_name, sheep_x, sheep_y)
                        max_distance = distance_to_pen

            if furthest_sheep:
                # Chase the furthest sheep
                _, sheep_x, sheep_y = furthest_sheep

                # Calculate the angle (theta) the sheep needs to head to reach the pen
                delta_x = self.pen_center_x - sheep_x
                delta_y = self.pen_center_y - sheep_y
                theta_to_pen = math.atan2(delta_y, delta_x)

                # Calculate the angle (theta) away from the wolf to position behind the sheep
                delta_wolf_x = sheep_x - wolf_pose["x"]
                delta_wolf_y = sheep_y - wolf_pose["y"]
                theta_away_from_wolf = math.atan2(delta_wolf_y, delta_wolf_x)

                # Combine the two directions to guide the sheep to the pen
                combined_theta = (theta_to_pen + theta_away_from_wolf) / 2

                # Position the wolf behind the sheep
                distance_behind = 5.0  # Distance to stay behind the sheep
                behind_x = sheep_x - distance_behind * math.cos(combined_theta)
                behind_y = sheep_y - distance_behind * math.sin(combined_theta)

                # Move the wolf towards the position behind the sheep
                direction_x = behind_x - wolf_pose["x"]
                direction_y = behind_y - wolf_pose["y"]
                move_length = math.hypot(direction_x, direction_y)
                if move_length > 0:
                    wolf_pose["x"] += (direction_x / move_length) * 0.5
                    wolf_pose["y"] += (direction_y / move_length) * 0.5
            else:
                # All sheep are in the pen; return to the wolf pen
                direction_x = self.wolf_pen_center_x - wolf_pose["x"]
                direction_y = self.wolf_pen_center_y - wolf_pose["y"]
                move_length = math.hypot(direction_x, direction_y)
                if move_length > 0:
                    wolf_pose["x"] += (direction_x / move_length) * 0.5
                    wolf_pose["y"] += (direction_y / move_length) * 0.5
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
