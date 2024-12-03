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
        # Pen position from master_node.py
        pen_x_min = 25.0 - 10.0  # grid[0] - pen_size
        pen_y_min = 25.0 - 10.0  # grid[1] - pen_size
        pen_x_max = 25.0
        pen_y_max = 25.0

        def is_in_pen(x, y):
            """Check if a position is inside the pen."""
            return pen_x_min <= x <= pen_x_max and pen_y_min <= y <= pen_y_max

        # Prevent the wolf from entering the pen
        if is_in_pen(wolf_pose["x"], wolf_pose["y"]):
            # Move the wolf out of the pen if inside
            if wolf_pose["x"] < pen_x_min:
                wolf_pose["x"] = pen_x_min - 1.0
            elif wolf_pose["x"] > pen_x_max:
                wolf_pose["x"] = pen_x_max + 1.0
            if wolf_pose["y"] < pen_y_min:
                wolf_pose["y"] = pen_y_min - 1.0
            elif wolf_pose["y"] > pen_y_max:
                wolf_pose["y"] = pen_y_max + 1.0
            return wolf_pose  # Skip further updates to avoid re-entering the pen

        if self.sheep_positions:
            # Keep track of sheep positions and ensure they stay in the pen
            for sheep_name, (sheep_x, sheep_y) in self.sheep_positions.items():
                if is_in_pen(sheep_x, sheep_y):
                    # Prevent sheep from leaving the pen
                    self.sheep_positions[sheep_name] = (
                        max(pen_x_min, min(sheep_x, pen_x_max)),
                        max(pen_y_min, min(sheep_y, pen_y_max)),
                    )

            # Find the furthest sheep outside the pen
            furthest_sheep, max_distance = None, -float('inf')
            for sheep_name, (sheep_x, sheep_y) in self.sheep_positions.items():
                if not is_in_pen(sheep_x, sheep_y):  # Only consider sheep outside the pen
                    distance_to_pen = math.sqrt((sheep_x - pen_x_min) ** 2 + (sheep_y - pen_y_min) ** 2)
                    if distance_to_pen > max_distance:
                        furthest_sheep, max_distance = (sheep_name, sheep_x, sheep_y), distance_to_pen

            if furthest_sheep:
                _, sheep_x, sheep_y = furthest_sheep

                # Calculate a position behind the sheep relative to the pen
                pen_center_x = (pen_x_min + pen_x_max) / 2
                pen_center_y = (pen_y_min + pen_y_max) / 2
                direction_to_pen_x = pen_center_x - sheep_x
                direction_to_pen_y = pen_center_y - sheep_y
                length = math.sqrt(direction_to_pen_x ** 2 + direction_to_pen_y ** 2)

                # Position the wolf behind the sheep (opposite to pen direction)
                behind_x = sheep_x - (direction_to_pen_x / length) * 1.0  # Offset to be "behind"
                behind_y = sheep_y - (direction_to_pen_y / length) * 1.0

                # Move the wolf towards the position behind the sheep
                direction_x = behind_x - wolf_pose["x"]
                direction_y = behind_y - wolf_pose["y"]
                move_length = math.sqrt(direction_x ** 2 + direction_y ** 2)

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
