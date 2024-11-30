import rclpy
from rclpy.node import Node
from sheep_simulation_interfaces.msg import EntityPose
from sheep_simulation_interfaces.srv import EntitySpawn
import random

class WolfSimulationNode(Node):
    def __init__(self):
        super().__init__('wolf')

        # Timer for wolf logic
        self.timer = self.create_timer(0.5, self.update_wolf)

        # Initial positions
        self.wolf_position = [0.0, 0.0]
        self.wolf_position = None  # Initially unknown

        # List tracking all wolf
        self.wolf = [
            # {
            #     name : "",
            #     pose : {x,y,theta},
            #     marker: None
            # }
        ]

        # Services
        self.wolf_spawn_service = self.create_service(EntitySpawn, "sheep_simulation/wolf/spawn", self.wolf_spawn_callback)

        # Publishers and subscribers
        self.wolf_position_publisher = self.create_publisher(EntityPose, 'sheep_simulation/wolf/pose', 10)

        # self.wolf_position_subscription = self.create_subscription(
        #     EntityPose, 'wolf_position', self.wolf_position_callback, 10
        # )

    def wolf_spawn_callback(self, request, response):
        try:
            self.get_logger().info(f"Incoming spawn request:")
            self.get_logger().info(f"name: {request.name}")
            self.get_logger().info(f"x: {request.x} y: {request.y}")

            wolf_obj = {
                "name" : request.name,
                "pose" : {
                    "x" : request.x,
                    "y" : request.y,
                    "theta" : request.theta
                },
                #"marker" : self.create_marker(request.name)
            }
            
            response.result = "ok"

            self.wolf.append(wolf_obj)
        except:
            response.result = "fail"

        return response

    def update_wolf(self):    
        for wolf in self.wolf:
            # update position
            wolf["pose"] = self.update_wolf_position(wolf["pose"])

            # publish position
            self.publish_wolf_position(wolf)


    def update_wolf_position(self, wolf_pose):
        # random walk
        wolf_pose = self.random_walk(wolf_pose)

        # prevent wolf moving outside of grid
        wolf_pose["x"] = max(-25.0, min(wolf_pose["x"], 25.0))
        wolf_pose["y"] = max(-25.0, min(wolf_pose["y"], 25.0))

        return wolf_pose
    
    def publish_wolf_position(self, wolf):
        position_msg = EntityPose()
        position_msg.name = wolf["name"]
        position_msg.x = wolf["pose"]["x"]
        position_msg.y = wolf["pose"]["y"]
        position_msg.theta = wolf["pose"]["theta"]

        self.wolf_position_publisher.publish(position_msg)

    def random_walk(self, pose):
        # Random movement
        return {
            "x" : pose["x"] + random.uniform(-0.5, 0.5),
            "y" : pose["y"] + random.uniform(-0.5, 0.5),
            "theta" : pose["theta"]
        }

    def wolf_position_callback(self, msg):
        # Update wolf's position when a message is received
        self.wolf_position = [msg.x, msg.y]
        self.get_logger().info(f"Received wolf position: {self.wolf_position}")


def main(args=None):
    rclpy.init(args=args)
    node = WolfSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()
