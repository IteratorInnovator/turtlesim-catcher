import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
import random
import math

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner_node")
        self.counter = 1
        self.spawn_turtle_timer = self.create_timer(2.5, self.spawn_turtle)
        self.spawn_turtle_client = self.create_client(Spawn, "spawn")


    def spawn_turtle(self):
        if not self.spawn_turtle_client.wait_for_service(5.0):
            self.get_logger().warn("/spawn service server unavailable")
            return
        
        request = Spawn.Request()
        request.name = f"catch_me_turtle_{self.counter}"
        self.counter += 1

        # Assign random x, y coordinates of turtle
        request.x = random.uniform(0, 11)
        request.y = random.uniform(0, 11)

        # Assign random orientation (in angle radians) of turtle
        request.theta = random.uniform(0, 2 * math.pi)
        future = self.spawn_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle, request=request))


    def callback_spawn_turtle(self, future, request):
        self.get_logger().info(f"{future.result().name} spawned succesfully.")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
