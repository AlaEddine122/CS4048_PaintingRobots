import rclpy
from rclpy.node import Node
from sheep_simulation_interfaces.msg import EntityPose
from sheep_simulation_interfaces.srv import EntitySpawn
import random

class SheepSimulationNode(Node):
    def __init__(self):
        super().__init__('sheep_simulation_node')

        # Timer for sheep logic
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
        self.sheep_spawn_service = self.create_service(EntitySpawn, "sheep_simulation/sheep/spawn", self.sheep_spawn_callback)

        # Publishers and subscribers
        self.sheep_position_publisher = self.create_publisher(EntityPose, 'sheep_simulation/sheep/pose', 10)

        # self.wolf_position_subscription = self.create_subscription(
        #     EntityPose, 'wolf_position', self.wolf_position_callback, 10
        # )

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
                }
            }
            
            response.result = "ok"

            self.sheep.append(sheep_obj)
        except:
            response.result = "fail"

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

            # publish position
            self.publish_sheep_position(sheep)

            # update marker
            #sheep["marker"].pose.position.x = sheep["pose"]["x"]
            #sheep["marker"].pose.position.y = sheep["pose"]["y"]

            # publish marker
            #self.sheep_marker_publisher.publish(sheep["marker"])

    def update_sheep_position(self, sheep_pose):
        # random walk
        sheep_pose = self.random_walk(sheep_pose)

        # prevent sheep moving outside of grid
        sheep_pose["x"] = max(-25.0, min(sheep_pose["x"], 25.0))
        sheep_pose["y"] = max(-25.0, min(sheep_pose["y"], 25.0))

        return sheep_pose
    
    def publish_sheep_position(self, sheep):
        position_msg = EntityPose()
        position_msg.name = sheep["name"]
        position_msg.x = sheep["pose"]["x"]
        position_msg.y = sheep["pose"]["y"]
        position_msg.theta = sheep["pose"]["theta"]

        self.sheep_position_publisher.publish(position_msg)

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
        return {
            "x" : pose["x"] + random.uniform(0, 2.5),
            "y" : pose["y"] + random.uniform(-0.5, 0.5),
            "theta" : pose["theta"]
        }

    def wolf_position_callback(self, msg):
        # Update wolf's position when a message is received
        self.wolf_position = [msg.x, msg.y]
        self.get_logger().info(f"Received wolf position: {self.wolf_position}")


def main(args=None):
    rclpy.init(args=args)
    node = SheepSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
