import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sheep_simulation_interfaces.msg import EntityPose
from sheep_simulation_interfaces.srv import EntitySpawn
import random

class MasterSimulationNode(Node):
    def __init__(self):
        super().__init__('master_simulation_node')

        # coords defining x,y edges of square grid
        self.grid = [25, 25]

        # simulation markers
        self.sheep_markers = {}
        self.sheep_marker_publisher = self.create_publisher(Marker, 'sheep_simulation/simulation/sheep_marker', 10)
        self.wolf_markers = {}
        self.wolf_marker_publisher = self.create_publisher(Marker, 'sheep_simulation/simulation/wolf_marker', 10)

        self.pen_marker_publisher = self.create_publisher(Marker, 'sheep_simulation/simulation/pen', 10)

        # clients to spawn entities
        self.sheep_spawn_client = self.create_client(EntitySpawn, 'sheep_simulation/sheep/spawn')
        while not self.sheep_spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for sheep /spawn service...')
        self.sheep_spawn_request = EntitySpawn.Request()

        self.wolf_spawn_client = self.create_client(EntitySpawn, 'sheep_simulation/wolf/spawn')
        while not self.wolf_spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for wolf /spawn service...')
        self.wolf_spawn_request = EntitySpawn.Request()

        # subcribe to entity position topics
        self.sheep_position_subscription = self.create_subscription(
            EntityPose, 'sheep_simulation/sheep/pose', self.sheep_position_callback, 10
        )
        self.wolf_position_subscription = self.create_subscription(
            EntityPose, 'sheep_simulation/wolf/pose', self.wolf_position_callback, 10
        )

        # publish sheep and wolf pen
        self.pen_size = 10.0
        self.pen_marker_publisher.publish(self.create_pen_marker("sheep_pen", size=self.pen_size))
        self.pen_marker_publisher.publish(self.create_pen_marker("wolf_pen", size=self.pen_size/2))

        # spawn 4 sheep
        for i in range(4):
            self.spawn_sheep(f"sheep{i+1}")
        
        # spawn 1 wolf:
        self.spawn_wolf("wolf1")

    def in_sheep_pen(self, x, y):
        return (x >= self.grid[0] - self.pen_size) and (y >= self.grid[1] - self.pen_size)

    def spawn_sheep(self, name, x=0.0, y=0.0, theta=0.0):
        # create request
        self.sheep_spawn_request.name = name
        self.sheep_spawn_request.x = x
        self.sheep_spawn_request.y = y
        self.sheep_spawn_request.theta = theta

        future = self.sheep_spawn_client.call_async(self.sheep_spawn_request)

        # create marker
        marker = self.create_marker("sheep", name)
        self.sheep_markers[name] = marker
        self.sheep_marker_publisher.publish(marker)

    def spawn_wolf(self, name):
        # spawn at random point inside wolf pen
        x = random.uniform(-self.grid[0], (-self.grid[0] + self.pen_size/2))
        y = random.uniform((self.grid[1] - self.pen_size/2), self.grid[1])
        theta = 0.0

        # create request
        self.wolf_spawn_request.name = name
        self.wolf_spawn_request.x = x
        self.wolf_spawn_request.y = y
        self.wolf_spawn_request.theta = theta

        future = self.wolf_spawn_client.call_async(self.wolf_spawn_request)

        # create marker
        marker = self.create_marker("wolf", name)
        self.wolf_markers[name] = marker
        self.wolf_marker_publisher.publish(marker)

    def sheep_position_callback(self, response):
        # update marker
        if self.sheep_markers[response.name]:
            self.sheep_markers[response.name].pose.position.x = response.x
            self.sheep_markers[response.name].pose.position.y = response.y

            self.sheep_marker_publisher.publish(self.sheep_markers[response.name])

        if self.in_sheep_pen(response.x, response.y):
            self.get_logger().info(f"{response.name} in pen")
    
    def wolf_position_callback(self, response):
        # update marker
        if self.wolf_markers[response.name]:
            self.wolf_markers[response.name].pose.position.x = response.x
            self.wolf_markers[response.name].pose.position.y = response.y

            self.wolf_marker_publisher.publish(self.wolf_markers[response.name])

        if self.in_sheep_pen(response.x, response.y):
            self.get_logger().info(f"{response.name} in pen")

    def create_marker(self, entity_type, name):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = name
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0 
        
        if entity_type == "sheep": # green for sheep
            marker.color.r = 0.0 
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif entity_type == "wolf": # red for wolf
            marker.color.r = 1.0 
            marker.color.g = 0.0
            marker.color.b = 0.0

        return marker

    def create_pen_marker(self, name, size=10.0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = name
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = size
        marker.scale.y = size

        if name == "sheep_pen":
            marker.pose.position.x = self.grid[0] - (size/2)
            marker.pose.position.y = self.grid[1] - (size/2)
            marker.pose.position.z = 0.0
            marker.scale.z = 0.1
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        elif name == "wolf_pen":
            marker.pose.position.x = -self.grid[0] + (size/2)
            marker.pose.position.y = self.grid[1] - (size/2)
            marker.pose.position.z = 0.0
            marker.scale.z = 0.1
            marker.color.a = 0.5
            marker.color.r = 0.5
            marker.color.g = 0.0
            marker.color.b = 0.0
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = MasterSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()

