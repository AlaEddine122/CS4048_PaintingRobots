import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class MasterSimulationNode(Node):
    def __init__(self):
        super().__init__('master_simulation_node')

        # coords defining edge of square grid
        grid = [-50, 50]

        # create clients to spawn entities
        self.sheep_spawn_client = self.create_client(Spawn, 'sheep_spawn')
        while not self.sheep_spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for sheep /spawn service...')
        self.sheep_spawn_request = Spawn.Request()

        # self.wolf_spawn_client = self.create_client(Spawn, 'wolf_spawn')
        # while not self.wolf_spawn_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for wolf /spawn service...')
        # self.wolf_spawn_request = Spawn.Request()

        self.spawn_sheep_test()

    def spawn_sheep_test(self):
        self.sheep_spawn_request.name = "testsheep"
        self.sheep_spawn_request.x = 0.0
        self.sheep_spawn_request.y = 0.0
        self.sheep_spawn_request.theta = 0.0

        future = self.sheep_spawn_client.call_async(self.sheep_spawn_request)
        



def main(args=None):
    rclpy.init(args=args)
    node = MasterSimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()

