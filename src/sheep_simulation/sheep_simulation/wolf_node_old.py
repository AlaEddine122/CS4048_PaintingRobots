import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math

class WolfNode(Node):
    def __init__(self):
        super().__init__('wolf_simulation_node')

        # Wolf state
        self.wolf_position = [5.0, 5.0]
        self.pen_position = [10.0, 10.0]
        self.pen_radius = 1.0

        # Subscribing to sheep's position
        self.sheep_subscription = self.create_subscription(
            Point, 'sheep_position', self.sheep_callback, 10)

        # Publisher for wolf's position
        self.wolf_publisher = self.create_publisher(Point, 'wolf_position', 10)
        
        # Marker publisher for RViz2
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker_wolf', 10)

        # Timer for updating wolf's position
        self.timer = self.create_timer(0.5, self.update_wolf_position)

        self.sheep_position = None  # To store the sheep's last known position

    def sheep_callback(self, msg):
        self.sheep_position = [msg.x, msg.y]

    def update_wolf_position(self):
        if self.sheep_position:
            # Calculate trajectory towards sheep and pen
            direction_to_sheep_x = self.sheep_position[0] - self.wolf_position[0]
            direction_to_sheep_y = self.sheep_position[1] - self.wolf_position[1]
            angle_to_sheep = math.atan2(direction_to_sheep_y, direction_to_sheep_x)
            
            # Move towards sheep, aiming to herd it towards the pen
            step_size = 0.3
            self.wolf_position[0] += math.cos(angle_to_sheep) * step_size
            self.wolf_position[1] += math.sin(angle_to_sheep) * step_size

            # Publish updated wolf position
            wolf_msg = Point()
            wolf_msg.x = self.wolf_position[0]
            wolf_msg.y = self.wolf_position[1]
            wolf_msg.z = 0.0
            self.wolf_publisher.publish(wolf_msg)
         # Publish visualization marker
            self.publish_wolf_marker()

            self.get_logger().info(f"Wolf moved to {self.wolf_position}")

    def publish_wolf_marker(self):
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
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    wolf_node = WolfNode()
    rclpy.spin(wolf_node)
    rclpy.shutdown()