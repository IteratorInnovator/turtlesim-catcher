import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
import random
import math
from custom_interfaces.msg import Turtle
from custom_interfaces.msg import TurtleArray
from custom_interfaces.srv import CatchTurtle
from functools import partial


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner_node")
        self.counter = 1
        self.alive_turtles= TurtleArray()
        self.declare_parameter("spawn_rate", 1.5)

        # Spawn a turtle every _ seconds
        self.spawn_turtle_timer = self.create_timer(self.get_parameter("spawn_rate").value, self.spawn_turtle)

        # Client that sends Spawn request to the /spawn service
        self.spawn_turtle_client = self.create_client(Spawn, "spawn")

        # Publisher that publishes the self.alives_turtles to the /alive_turtle topic
        self.publisher = self.create_publisher(TurtleArray, "alive_turtles", 10)

        # Service server that receives a CatchTurtle request from the /catch_turtle service
        self.catch_turtle_server = self.create_service(CatchTurtle, "catch_turtle", self.kill_turtle)
        
        # Service client that sends a Kill request to the /kill service
        self.kill_turtle_client = self.create_client(Kill, "kill")


    def spawn_turtle(self):
        """
        Callback function to the self.spawn_turtle_timer.
        Spawns a new turtle at a random location and with a random orientation.
        Generates random x, y coordinates and theta angle and sends a Spawn request to the /spawn service.
        """
        if not self.spawn_turtle_client.wait_for_service(5.0):
            self.get_logger().warn("/spawn service server unavailable")
            return
        
        request = Spawn.Request()
        request.name = f"catch_me_turtle_{self.counter}"
        self.counter += 1

        # Assign random x, y coordinates of turtle
        request.x = random.uniform(0.0, 11.0)
        request.y = random.uniform(0.0, 11.0)

        # Assign random orientation (in angle radians) of turtle
        request.theta = random.uniform(0, 2 * math.pi)

        # Send a Spawn request to /spawn service server
        future = self.spawn_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle, request=request))


    def callback_spawn_turtle(self, future, request):

        # Append the newly spawned turtle to the self.alive_turtles array
        spawned_turtle = Turtle()
        spawned_turtle.name, spawned_turtle.x, spawned_turtle.y, spawned_turtle.theta = request.name, request.x, request.y, request.theta
        self.alive_turtles.turtles.append(spawned_turtle)
        self.get_logger().info(f"{future.result().name} spawned succesfully.")
        self.publisher.publish(self.alive_turtles)


    def kill_turtle(self, request : CatchTurtle.Request, response : CatchTurtle.Response):
        """
        Callback function when self.catch_turtle_server receives a CatchTurtle request.
        Sends a request to the /kill service to kill the target turtle.
        Update response.success to True if target turtle is killed successfully, vice versa.
        """

        while not self.kill_turtle_client.wait_for_service(10.0):
            self.get_logger().warn("/kill service is not ready!")

        kill_request = Kill.Request()
        kill_request.name = request.name

        # Call /kill service
        future = self.kill_turtle_client.call_async(kill_request)

        # self.alives_turtle array still needs to be updated to reflect the deletion of the target turtle, hence the need for the done_callback
        future.add_done_callback(partial(self.callback_kill_turtle, kill_request=kill_request))

        response.success = True
        return response
    

    def callback_kill_turtle(self, future, kill_request):
        """
        Deletes the target turtle in the self.alives_turtles array.
        Note: /kill service server returns an empty response
        """
        for i, turtle in enumerate(self.alive_turtles.turtles):
            if kill_request.name == turtle.name:
                del self.alive_turtles.turtles[i]
                self.publisher.publish(self.alive_turtles)
                break


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
