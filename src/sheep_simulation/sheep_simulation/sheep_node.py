import rclpy
from rclpy.node import Node
from sheep_simulation_interfaces.msg import EntityPose
from sheep_simulation_interfaces.srv import EntitySpawn, Grid
import math
import random


class SheepSimulationNode(Node):
    def __init__(self):
        super().__init__('sheep_simulation_node')

        # Timer for sheep logic
        self.timer = self.create_timer(0.5, self.update_simulation)

        # List of all sheep
        self.sheep = []

        # Services
        self.sheep_spawn_service = self.create_service(EntitySpawn, "sheep_simulation/sheep/spawn", self.sheep_spawn_callback)

        # Clients
        self.grid_init_client = self.create_client(Grid, 'sheep_simulation/grid')
        while not self.grid_init_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for grid service...')

        # Publishers
        self.sheep_position_publisher = self.create_publisher(EntityPose, 'sheep_simulation/sheep/pose', 10)

        # Subscribers
        # self.grid_subscription = self.create_subscription(Grid, 'sheep_simulation/gtid', self.grid_initialisation_callback, 10)
        self.wolf_position_subscription = self.create_subscription( EntityPose, 'sheep_simulation/wolf/pose', self.wolf_position_callback, 10)

        # Tracking wolf positions
        self.wolf_positions = {}

        self.init_grid()

        # # Pen location (defined in master_node.py)
        # self.pen_x_min = 25.0 - 10.0
        # self.pen_y_min = 25.0 - 10.0
        # self.pen_x_max = 25.0
        # self.pen_y_max = 25.0
        # self.pen_center_x = (self.pen_x_min + self.pen_x_max) / 2
        # self.pen_center_y = (self.pen_y_min + self.pen_y_max) / 2

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

    def sheep_spawn_callback(self, request, response):
        try:
            sheep_obj = {
                "name": request.name,
                "pose": {
                    "x": request.x,
                    "y": request.y,
                    "theta": request.theta
                }
            }
            self.sheep.append(sheep_obj)
            response.result = "ok"
            self.get_logger().info(f"Spawned sheep: {request.name} at ({request.x}, {request.y})")
        except Exception as e:
            response.result = "fail"
            self.get_logger().error(f"Failed to spawn sheep: {e}")
        return response

    def wolf_position_callback(self, msg):
        # Track wolf positions by their names
        self.wolf_positions[msg.name] = (msg.x, msg.y)

    def grid_initialisation_callback(self, msg):
        self.grid = [
            [msg.xmin, msg.xmax],
            [msg.ymin, msg.ymax]
        ]

        self.pen_x_min = self.grid[0][1] - 10.0
        self.pen_y_min = self.grid[1][1] - 10.0
        self.pen_x_max = self.grid[0][1]
        self.pen_y_max = self.grid[1][1]
        self.pen_center_x = (self.pen_x_min + self.pen_x_max) / 2
        self.pen_center_y = (self.pen_y_min + self.pen_y_max) / 2

    def update_simulation(self):
        if not hasattr(self, "grid"):
            return
        
        for sheep in self.sheep:
            # Update the sheep's position
            sheep["pose"] = self.update_sheep_position(sheep["pose"])

            # Publish the updated position
            self.publish_sheep_position(sheep)

    def update_sheep_position(self, sheep_pose):
        
        def is_in_pen(x, y):
            """Check if a position is inside the pen."""
            return self.pen_x_min <= x <= self.pen_x_max and self.pen_y_min <= y <= self.pen_y_max

        if is_in_pen(sheep_pose["x"], sheep_pose["y"]):
            # If the sheep is in the pen, stop moving
            return sheep_pose

        if self.wolf_positions:
            # Calculate the closest wolf
            closest_wolf, min_distance = None, float('inf')
            for wolf_name, (wolf_x, wolf_y) in self.wolf_positions.items():
                distance = math.sqrt((sheep_pose["x"] - wolf_x) ** 2 + (sheep_pose["y"] - wolf_y) ** 2)
                if distance < min_distance:
                    closest_wolf, min_distance = (wolf_x, wolf_y), distance

            if closest_wolf and min_distance < 10.0:  # Wolf is close enough to influence the sheep
                wolf_x, wolf_y = closest_wolf

                # Calculate the angle (theta) to the pen
                delta_x = self.pen_center_x - sheep_pose["x"]
                delta_y = self.pen_center_y - sheep_pose["y"]
                theta_to_pen = math.atan2(delta_y, delta_x)

                # Calculate the angle (theta) away from the wolf
                delta_wolf_x = sheep_pose["x"] - wolf_x
                delta_wolf_y = sheep_pose["y"] - wolf_y
                theta_away_from_wolf = math.atan2(delta_wolf_y, delta_wolf_x)

                # Combine the two directions to encourage movement towards the pen but away from the wolf
                combined_theta = (theta_to_pen + theta_away_from_wolf) / 2

                # Move the sheep in the combined direction
                sheep_pose["x"] += math.cos(combined_theta) * 0.5
                sheep_pose["y"] += math.sin(combined_theta) * 0.5
            else:
                # Wolf is far; wander randomly
                sheep_pose = self.random_walk(sheep_pose)
        else:
            # No wolves detected; wander randomly
            sheep_pose = self.random_walk(sheep_pose)

        # Keep sheep within grid boundaries
        sheep_pose["x"] = max(-25.0, min(sheep_pose["x"], 25.0))
        sheep_pose["y"] = max(-25.0, min(sheep_pose["y"], 25.0))
        return sheep_pose


    def random_walk(self, pose):
        return {
            "x": pose["x"] + random.uniform(-0.5, 0.5),
            "y": pose["y"] + random.uniform(-0.5, 0.5),
            "theta": pose["theta"]
        }

    def publish_sheep_position(self, sheep):
        position_msg = EntityPose()
        position_msg.name = sheep["name"]
        position_msg.x = sheep["pose"]["x"]
        position_msg.y = sheep["pose"]["y"]
        position_msg.theta = sheep["pose"]["theta"]
        self.sheep_position_publisher.publish(position_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SheepSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
