import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import random

class SheepSimulationNode(Node):
    def __init__(self):
        super().__init__('sheep_simulation_node')

        # Timer for sheep movement
        self.timer = self.create_timer(0.5, self.update_simulation)

        # Initial positions
        self.sheep_position = [0.0, 0.0]
        self.wolf_position = [5.0, 5.0]

        # Marker publishers
        self.sheep_marker_publisher = self.create_publisher(Marker, 'visualization_marker_sheep', 10)
        self.wolf_marker_publisher = self.create_publisher(Marker, 'visualization_marker_wolf', 10)

    def update_simulation(self):
        # Sheep movement logic
        wolf_distance = ((self.sheep_position[0] - self.wolf_position[0]) ** 2 +
                         (self.sheep_position[1] - self.wolf_position[1]) ** 2) ** 0.5
        if wolf_distance < 10.0:  # Sheep runs away if wolf is close
            self.run_away()
        else:
            self.random_walk()

        # Publish markers for both sheep and wolf
        self.publish_sheep_marker()
        self.publish_wolf_marker()

    def run_away(self):
        # Move sheep away from wolf
        direction_x = self.sheep_position[0] - self.wolf_position[0]
        direction_y = self.sheep_position[1] - self.wolf_position[1]
        length = (direction_x**2 + direction_y**2) ** 0.5

        # Normalize and move away
        self.sheep_position[0] += (direction_x / length) * 0.5
        self.sheep_position[1] += (direction_y / length) * 0.5
        self.get_logger().info(f"Sheep ran to {self.sheep_position}")

    def random_walk(self):
        # Random movement
        self.sheep_position[0] += random.uniform(-0.5, 0.5)
        self.sheep_position[1] += random.uniform(-0.5, 0.5)
        self.get_logger().info(f"Sheep randomly moved to {self.sheep_position}")

    def publish_sheep_marker(self):
        # Sheep marker (green sphere)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sheep"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.sheep_position[0]
        marker.pose.position.y = self.sheep_position[1]
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0  # Green for sheep
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.sheep_marker_publisher.publish(marker)

    def publish_wolf_marker(self):
        # Wolf marker (red sphere)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wolf"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.wolf_position[0]
        marker.pose.position.y = self.wolf_position[1]
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 1.0  # Red for wolf
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.wolf_marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SheepSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
