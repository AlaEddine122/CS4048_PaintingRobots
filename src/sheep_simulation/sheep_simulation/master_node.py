import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sheep_simulation_interfaces.msg import EntityPose
from sheep_simulation_interfaces.srv import EntitySpawn
import random

class MasterSimulationNode(Node):
    def __init__(self):
        super().__init__('master_simulation_node')

        # coords defining edge of square grid
        grid = [-25, 25]

        # simulation markers
        self.sheep_markers = {}
        self.sheep_marker_publisher = self.create_publisher(Marker, 'sheep_simulation/simulation/sheep_marker', 10)
        self.wolf_markers = {}
        self.wolf_marker_publisher = self.create_publisher(Marker, 'sheep_simulation/simulation/wolf_marker', 10)

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

        # spawn 4 sheep
        for i in range(4):
            self.spawn_sheep(f"sheep{i+1}")
        
        # spawn 1 wolf:
        self.spawn_wolf("wolf1")

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

    def spawn_wolf(self, name, x=0.0, y=0.0, theta=0.0):
        x = random.uniform(-25.0, 25.0)
        y = random.uniform(-25.0, 25.0)

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
    
    def wolf_position_callback(self, response):
        # update marker
        if self.wolf_markers[response.name]:
            self.wolf_markers[response.name].pose.position.x = response.x
            self.wolf_markers[response.name].pose.position.y = response.y

            self.wolf_marker_publisher.publish(self.wolf_markers[response.name])


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


def main(args=None):
    rclpy.init(args=args)
    node = MasterSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()

