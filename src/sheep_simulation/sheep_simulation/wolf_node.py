import rclpy
from rclpy.node import Node
from sheep_simulation_interfaces.msg import EntityPose
from sheep_simulation_interfaces.srv import EntitySpawn
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

        # Publishers
        self.wolf_position_publisher = self.create_publisher(EntityPose, 'sheep_simulation/wolf/pose', 10)

        # Subscribers
        self.sheep_position_subscription = self.create_subscription(
            EntityPose, 'sheep_simulation/sheep/pose', self.sheep_position_callback, 10
        )

        # Tracking sheep positions
        self.sheep_positions = {}

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
        for wolf in self.wolves:
            # Update the wolf's position
            wolf["pose"] = self.update_wolf_position(wolf["pose"])

            # Publish the updated position
            self.publish_wolf_position(wolf)

    def update_wolf_position(self, wolf_pose):
        if self.sheep_positions:
            # Find the closest sheep
            closest_sheep, min_distance = None, float('inf')
            for sheep_name, (sheep_x, sheep_y) in self.sheep_positions.items():
                distance = math.sqrt((wolf_pose["x"] - sheep_x) ** 2 + (wolf_pose["y"] - sheep_y) ** 2)
                if distance < min_distance:
                    closest_sheep, min_distance = (sheep_x, sheep_y), distance

            if closest_sheep:
                direction_x = closest_sheep[0] - wolf_pose["x"]
                direction_y = closest_sheep[1] - wolf_pose["y"]
                length = math.sqrt(direction_x**2 + direction_y**2)

                # Move towards the closest sheep
                wolf_pose["x"] += (direction_x / length) * 0.5
                wolf_pose["y"] += (direction_y / length) * 0.5

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
