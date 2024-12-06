import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from sheep_simulation_interfaces.msg import EntityPose, EntityPoseArray
from sheep_simulation_interfaces.srv import EntitySpawn, Grid
import random
import math


class MasterSimulationNode(Node):
    def __init__(self):
        super().__init__('master_simulation_node')

        # simulation markers
        self.sheep_markers = {}
        self.sheep_marker_publisher = self.create_publisher(MarkerArray, 'sheep_simulation/simulation/sheep_markers', 10)
        self.wolf_markers = {}
        self.wolf_marker_publisher = self.create_publisher(MarkerArray, 'sheep_simulation/simulation/wolf_markers', 10)

        self.pen_marker_publisher = self.create_publisher(MarkerArray, 'sheep_simulation/simulation/pen_markers', 10)

        # Services
        self.grid_init_service = self.create_service(Grid, "sheep_simulation/grid", self.grid_init_callback)

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
        self.sheep_position_subscription = self.create_subscription(EntityPoseArray, 'sheep_simulation/sheep/pose', self.sheep_position_callback, 10)
        self.wolf_position_subscription = self.create_subscription(EntityPoseArray, 'sheep_simulation/wolf/pose', self.wolf_position_callback, 10)

        # create grid
        grid_size = 50.0
        self.grid = self.create_grid(size=grid_size)

        # create pens
        self.pen_size = 10.0
        pen_markers = []
        self.pen_markers_msg = MarkerArray()

        pen_markers.append(self.create_pen_marker("sheep_pen", size=self.pen_size))
        pen_markers.append(self.create_pen_marker("wolf_pen1", size=self.pen_size / 2))
        pen_markers.append(self.create_pen_marker("wolf_pen2", size=self.pen_size / 2))

        self.pen_markers_msg.markers = pen_markers
        self.create_timer(0.5, self.update_markers)
        #self.pen_marker_publisher.publish(pen_markers_msg)

        # spawn 6 sheep in 2 groups of 3
        self.spawn_sheep_group("group1", center_x=-(grid_size/3), center_y=(grid_size/3))
        self.spawn_sheep_group("group2", center_x=(grid_size/3), center_y=-(grid_size/3))

        # spawn 2 wolves in respective pens
        wolf_spawns = [
            ["wolf1", self.grid[0][0]+self.pen_size/4, self.grid[1][1]-self.pen_size/4],
            ["wolf2", self.grid[0][0]+self.pen_size/4, self.grid[1][0]+self.pen_size/4]
        ]
        self.spawn_wolf_group(wolf_spawns)
        # self.spawn_wolf("wolf1", x=self.grid[0][0]+self.pen_size/4, y=self.grid[1][1]-self.pen_size/4)
        # self.spawn_wolf("wolf2", x=self.grid[0][0]+self.pen_size/4, y=self.grid[1][0]+self.pen_size/4)

    def create_grid(self, size):
        grid = [
            [-(size/2), (size/2)], #xmin, xmax
            [-(size/2), (size/2)]  #ymin, ymax
        ]

        return grid

    def spawn_sheep_group(self, group_name, center_x, center_y):
        spawn_array = []
        for i in range(50):
            entity = EntityPose()

            entity.name = f"{group_name}_sheep{i+1}"
            # entity.x = random.uniform(center_x - 2.0, center_x + 2.0)
            # entity.y = random.uniform(center_y - 2.0, center_y + 2.0)
            entity.x = random.uniform(self.grid[0][0], self.grid[0][1])
            entity.y = random.uniform(self.grid[1][0], self.grid[1][1])

            spawn_array.append(entity)
            # self.spawn_sheep(name, x, y)
            
            # Create markers for visualization
            marker = self.create_marker("sheep", entity.name)
            marker.pose.position.x = entity.x
            marker.pose.position.y = entity.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.z = math.sin(entity.theta / 2)
            marker.pose.orientation.w = math.cos(entity.theta / 2)

            self.sheep_markers[entity.name] = marker
        
        self.sheep_spawn_request.spawn_entities = spawn_array

        future = self.sheep_spawn_client.call_async(self.sheep_spawn_request)

        rclpy.spin_until_future_complete(self, future)

    def spawn_wolf_group(self, wolves):
        spawn_array = []
        for wolf in wolves:
            entity = EntityPose()

            entity.name = f"{wolf[0]}"
            entity.x = wolf[1]
            entity.y = wolf[2]

            spawn_array.append(entity)
            
            # Create markers for visualization
            marker = self.create_marker("wolf", entity.name)
            marker.pose.position.x = entity.x
            marker.pose.position.y = entity.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.z = math.sin(entity.theta / 2)
            marker.pose.orientation.w = math.cos(entity.theta / 2)

            self.wolf_markers[entity.name] = marker
        
        self.wolf_spawn_request.spawn_entities = spawn_array

        future = self.wolf_spawn_client.call_async(self.wolf_spawn_request)

        rclpy.spin_until_future_complete(self, future)


    def in_pen(self, x, y):
        return (x >= self.grid[0][1] - self.pen_size) and (y >= self.grid[1][1] - self.pen_size)

    def sheep_position_callback(self, response):
        msg = MarkerArray()
        markers = []
        for sheep in response.entity_positions:
            if self.sheep_markers[sheep.name]:
                self.sheep_markers[sheep.name].pose.position.x = sheep.x
                self.sheep_markers[sheep.name].pose.position.y = sheep.y
                # self.sheep_marker_publisher.publish(self.sheep_markers[sheep.name])
                markers.append(self.sheep_markers[sheep.name])

        msg.markers = markers
        self.sheep_marker_publisher.publish(msg)

    def wolf_position_callback(self, response):
        msg = MarkerArray()
        markers = []
        for wolf in response.entity_positions:
            if self.wolf_markers[wolf.name]:
                self.wolf_markers[wolf.name].pose.position.x = wolf.x
                self.wolf_markers[wolf.name].pose.position.y = wolf.y
                # self.sheep_marker_publisher.publish(self.sheep_markers[sheep.name])
                markers.append(self.wolf_markers[wolf.name])

        msg.markers = markers
        self.wolf_marker_publisher.publish(msg)

    def grid_init_callback(self, request, response):
        response.xmin = self.grid[0][0]
        response.xmax = self.grid[0][1]
        response.ymin = self.grid[1][0]
        response.ymax = self.grid[1][1]
        response.pensize = self.pen_size

        return response

    def create_marker(self, entity_type, name):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = name
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0

        if entity_type == "sheep":  # Green for sheep
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif entity_type == "wolf":  # Red for wolf
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
        marker.scale.z = 0.1

        if name == "sheep_pen":
            marker.pose.position.x = self.grid[0][1] - (size/2)
            marker.pose.position.y = self.grid[1][1] - (size/2)
            marker.pose.position.z = 0.0
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

        elif name == "wolf_pen1":
            marker.pose.position.x = self.grid[0][0] + (size/2)
            marker.pose.position.y = self.grid[1][1] - (size/2)
            marker.pose.position.z = 0.0
            marker.color.a = 0.5
            marker.color.r = 0.5
            marker.color.g = 0.0
            marker.color.b = 0.0

        elif name == "wolf_pen2":
            marker.pose.position.x = self.grid[0][0] + (size/2)
            marker.pose.position.y = self.grid[0][0] + (size/2)
            marker.pose.position.z = 0.0
            marker.color.a = 0.5
            marker.color.r = 0.5
            marker.color.g = 0.0
            marker.color.b = 0.0

        return marker

    def update_markers(self):
        self.pen_marker_publisher.publish(self.pen_markers_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MasterSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
