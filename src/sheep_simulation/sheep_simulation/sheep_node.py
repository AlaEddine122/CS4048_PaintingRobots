import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random

class SheepSimulationNode(Node):
    def __init__(self):
        super().__init__('sheep_simulation_node')

        # Timer for sheep movement
        self.timer = self.create_timer(0.5, self.update_simulation)

        # Initial positions
        self.sheep_position = [0.0, 0.0]
        self.wolf_position = None  # Initially unknown

        # List tracking all sheep
        self.sheep = [
            # {
            #     name : "",
            #     pose : {x,y,theta},
            #     marker: None
            # }
        ]

        # Services
        self.sheep_spawn_service = self.create_service(Spawn, "sheep_spawn", self.sheep_spawn_callback)

        # Publishers and subscribers
        self.sheep_position_publisher = self.create_publisher(Point, 'sheep_position', 10)
        self.wolf_position_subscription = self.create_subscription(
            Point, 'wolf_position', self.wolf_position_callback, 10
        )

        # Marker publisher
        self.sheep_marker_publisher = self.create_publisher(Marker, 'visualization_marker_sheep', 10)

    def sheep_spawn_callback(self, request, response):
        try:
            self.get_logger().info(f"Incoming spawn request:")
            self.get_logger().info(f"name: {request.name}")
            self.get_logger().info(f"x: {request.x} y: {request.y}")

            sheep_obj = {
                "name" : request.name,
                "pose" : {
                    "x" : request.x,
                    "y" : request.y,
                    "theta" : request.theta
                },
                "marker" : self.create_marker(request.name)
            }
            
            response.name = "ok"

            self.sheep.append(sheep_obj)
        except:
            response.name = "fail"

        return response

    def update_simulation(self):
        # if self.wolf_position:
        #     # Calculate distance between sheep and wolf
        #     wolf_distance = ((self.sheep_position[0] - self.wolf_position[0]) ** 2 +
        #                      (self.sheep_position[1] - self.wolf_position[1]) ** 2) ** 0.5
        #     if wolf_distance < 10.0:  # Sheep runs away if wolf is close
        #         self.run_away()
        #     else:
        #         self.random_walk()
        # else:
        #     # No wolf detected; sheep moves randomly
        #     self.random_walk()

        # # Publish sheep's position
        # self.publish_sheep_position()

        # # Publish sheep's marker
        # self.publish_sheep_marker()
        
        for sheep in self.sheep:
            # update position
            sheep["pose"] = self.update_sheep_position(sheep["pose"])

            # update marker
            sheep["marker"].pose.position.x = sheep["pose"]["x"]
            sheep["marker"].pose.position.y = sheep["pose"]["y"]

            # publish marker
            self.sheep_marker_publisher.publish(sheep["marker"])

    def update_sheep_position(self, sheep_pose):
        # random walk
        sheep_pose = self.random_walk(sheep_pose)

        # prevent sheep moving outside of grid
        sheep_pose["x"] = max(-25.0, min(sheep_pose["x"], 25.0))
        sheep_pose["y"] = max(-25.0, min(sheep_pose["y"], 25.0))

        return sheep_pose

    def run_away(self):
        # Move sheep away from wolf
        direction_x = self.sheep_position[0] - self.wolf_position[0]
        direction_y = self.sheep_position[1] - self.wolf_position[1]
        length = (direction_x**2 + direction_y**2) ** 0.5

        # Normalize and move away
        self.sheep_position[0] += (direction_x / length) * 0.5
        self.sheep_position[1] += (direction_y / length) * 0.5
        self.get_logger().info(f"Sheep ran to {self.sheep_position}")

    def random_walk(self, pose):
        # Random movement
        # self.sheep_position[0] += random.uniform(-0.5, 0.5)
        # self.sheep_position[1] += random.uniform(-0.5, 0.5)
        # self.get_logger().info(f"Sheep randomly moved to {self.sheep_position}")
        return {
            "x" : pose["x"] + random.uniform(0, 2.5),
            "y" : pose["y"] + random.uniform(-0.5, 0.5),
            "theta" : pose["theta"]
        }


    def wolf_position_callback(self, msg):
        # Update wolf's position when a message is received
        self.wolf_position = [msg.x, msg.y]
        self.get_logger().info(f"Received wolf position: {self.wolf_position}")

    def publish_sheep_position(self):
        # Publish the sheep's current position
        position_msg = Point()
        position_msg.x = self.sheep_position[0]
        position_msg.y = self.sheep_position[1]
        position_msg.z = 0.0
        self.sheep_position_publisher.publish(position_msg)
     
    def publish_sheep_marker(self, sheep):
        # Sheep marker (green sphere)
        marker = Marker()
        marker.header.frame_id = "map"
        #marker.header.stamp = self.get_clock().now().to_msg()
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
    
    def create_marker(self, sheep_name):
        # Sheep marker (green sphere)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = sheep_name
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0  # Green for sheep
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker

def main(args=None):
    rclpy.init(args=args)
    node = SheepSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
