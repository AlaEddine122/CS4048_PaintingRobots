import rclpy
from rclpy.node import Node
from sheep_simulation_interfaces.msg import EntityPose
from sheep_simulation_interfaces.srv import EntitySpawn
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

        # Publishers
        self.sheep_position_publisher = self.create_publisher(EntityPose, 'sheep_simulation/sheep/pose', 10)

        # Subscribers
        self.wolf_position_subscription = self.create_subscription(
            EntityPose, 'sheep_simulation/wolf/pose', self.wolf_position_callback, 10
        )

        # Tracking wolf positions
        self.wolf_positions = {}

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

    def update_simulation(self):
        for sheep in self.sheep:
            # Update the sheep's position
            sheep["pose"] = self.update_sheep_position(sheep["pose"])

            # Publish the updated position
            self.publish_sheep_position(sheep)

    def update_sheep_position(self, sheep_pose):
        if self.wolf_positions:
            # Calculate the closest wolf
            closest_wolf, min_distance = None, float('inf')
            for wolf_name, (wolf_x, wolf_y) in self.wolf_positions.items():
                distance = math.sqrt((sheep_pose["x"] - wolf_x) ** 2 + (sheep_pose["y"] - wolf_y) ** 2)
                if distance < min_distance:
                    closest_wolf, min_distance = (wolf_x, wolf_y), distance

            if closest_wolf and min_distance < 10.0:  # Wolf is too close; sheep runs away
                direction_x = sheep_pose["x"] - closest_wolf[0]
                direction_y = sheep_pose["y"] - closest_wolf[1]
                length = math.sqrt(direction_x**2 + direction_y**2)

                # Move away from the wolf
                sheep_pose["x"] += (direction_x / length) * 0.5
                sheep_pose["y"] += (direction_y / length) * 0.5
            else:
                # No nearby wolf; random movement
                sheep_pose = self.random_walk(sheep_pose)
        else:
            # No wolves detected; random movement
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
