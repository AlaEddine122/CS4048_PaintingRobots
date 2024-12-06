import rclpy
from rclpy.node import Node
from sheep_simulation_interfaces.msg import EntityPose, EntityPoseArray
from sheep_simulation_interfaces.srv import EntitySpawn, Grid
from sheep_simulation.PsuedoSheepSeperationPort import PsuedoSheep
import math
import random
import numpy as np


class SheepSimulationNode(Node):
    def __init__(self):
        super().__init__('sheep_simulation_node')
        # Timer for sheep logic
        self.timer = self.create_timer(0.1, self.update_simulation)

        # List of all sheep
        self.sheep = []

        # Services
        self.sheep_spawn_service = self.create_service(EntitySpawn, "sheep_simulation/sheep/spawn", self.sheep_spawn_callback)

        # Clients
        self.grid_init_client = self.create_client(Grid, 'sheep_simulation/grid')
        while not self.grid_init_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for grid service...')

        # Publishers
        self.sheep_position_publisher = self.create_publisher(EntityPoseArray, 'sheep_simulation/sheep/pose', 10)

        # Subscribers
        # self.grid_subscription = self.create_subscription(Grid, 'sheep_simulation/gtid', self.grid_initialisation_callback, 10)
        self.wolf_position_subscription = self.create_subscription(EntityPoseArray, 'sheep_simulation/wolf/pose', self.wolf_position_callback, 10)

        # Tracking wolf positions
        self.wolf_positions = {}

        self.init_grid()

        self.boids = PsuedoSheep(logger=self.get_logger())

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
            for sheep in request.spawn_entities:

                sheep_obj = {
                    "name": sheep.name,
                    "pose": {
                        "x": sheep.x,
                        "y": sheep.y,
                        "theta": sheep.theta
                    }
                }

                self.sheep.append(sheep_obj)
                response.result = "ok"
                self.get_logger().info(f"Spawned sheep: {sheep.name} at ({sheep.x}, {sheep.y})")
        except Exception as e:
            response.result = "fail"
            self.get_logger().error(f"Failed to spawn sheep: {e}")
        return response

    def wolf_position_callback(self, msg):
        for wolf in msg.entity_positions:
            self.wolf_positions[wolf.name] = (wolf.x, wolf.y)

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
        
        positions = []
        for sheep in self.sheep:
            if self.sheep_safe():
                sheep["pose"] = self.update_sheep_in_pen(sheep["pose"])
            else:
                sheep["pose"] = self.update_sheep_position(sheep["pose"])

                localsheep = np.array([
                    sheep["pose"]["x"],
                    sheep["pose"]["y"],
                    sheep["pose"]["theta"]
                ])
                globalsheep = np.array([
                    [sheep["pose"]["x"],
                    sheep["pose"]["y"],
                    sheep["pose"]["theta"]]
                    for sheep in self.sheep
                ])
                dx, dy, dtheta = self.boids.update_velocity(localsheep, globalsheep)

                sheep["pose"]["x"] += dx
                sheep["pose"]["y"] += dy
                sheep["pose"]["theta"] += dtheta

            sheep["pose"]["x"] = max(self.grid[0][0], min(sheep["pose"]["x"], self.grid[0][1]))
            sheep["pose"]["y"] = max(self.grid[1][0], min(sheep["pose"]["y"], self.grid[1][1]))

            entity = EntityPose()
            entity.name = sheep["name"]
            entity.x = sheep["pose"]["x"]
            entity.y = sheep["pose"]["y"]
            entity.theta = sheep["pose"]["theta"]
            positions.append(entity)
        
        self.publish_sheep_positions(positions)

    def sheep_safe(self):
        return all([(sheep["pose"]["x"] >= self.pen_x_min) and (sheep["pose"]["y"] >= self.pen_y_min) for sheep in self.sheep])
    
    def update_sheep_in_pen(self, sheep_pose):
        sheep_pose = self.random_walk(sheep_pose)

        sheep_pose["x"] = max(self.pen_x_min, sheep_pose["x"])
        sheep_pose["y"] = max(self.pen_x_min, sheep_pose["y"])

        return sheep_pose

    def update_sheep_position(self, sheep_pose):
        
        def is_in_pen(x, y):
            return self.pen_x_min <= x <= self.pen_x_max and self.pen_y_min <= y <= self.pen_y_max

        if is_in_pen(sheep_pose["x"], sheep_pose["y"]):
            return sheep_pose

        if self.wolf_positions:
            closest_wolf = min(
                self.wolf_positions.values(),
                key=lambda wolf_pos: math.hypot(wolf_pos[0] - sheep_pose["x"], wolf_pos[1] - sheep_pose["y"])
            )
            distance = math.hypot(closest_wolf[0] - sheep_pose["x"], closest_wolf[1] - sheep_pose["y"])

            if distance < 10.0:
                delta_x = self.pen_center_x - sheep_pose["x"]
                delta_y = self.pen_center_y - sheep_pose["y"]
                theta_to_pen = math.atan2(delta_y, delta_x)

                delta_wolf_x = sheep_pose["x"] - closest_wolf[0]
                delta_wolf_y = sheep_pose["y"] - closest_wolf[1]
                theta_away_from_wolf = math.atan2(delta_wolf_y, delta_wolf_x)

                combined_theta = (theta_to_pen + theta_away_from_wolf) / 2
                sheep_pose["x"] += math.cos(combined_theta) * 0.5
                sheep_pose["y"] += math.sin(combined_theta) * 0.5
            else:
                sheep_pose = self.random_walk(sheep_pose)
        else:
            sheep_pose = self.random_walk(sheep_pose)
        
        return sheep_pose

    def random_walk(self, pose):
        return {
            "x": pose["x"] + random.uniform(-0.5, 0.5),
            "y": pose["y"] + random.uniform(-0.5, 0.5),
            "theta": pose["theta"]
        }

    def publish_sheep_positions(self, positions):
        msg = EntityPoseArray()
        msg.entity_positions = positions
        self.sheep_position_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SheepSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
