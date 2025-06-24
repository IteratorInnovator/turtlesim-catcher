import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from custom_interfaces.msg import Turtle
from custom_interfaces.msg import TurtleArray
from custom_interfaces.srv import CatchTurtle
from functools import partial


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller_node")
        self.pose : Pose = None
        self.target_turtle : Turtle = None
        self.catch_closest_turtle_first = True

        # This subscriber will subscribe to the master turtle pose topic to retrieve the position of the master turtle
        self.pose_subscriber = self.create_subscription(Pose, "/master_turtle/pose", self.set_pose, 10)

        # This publish will publish instructions to the master turtle cmd_vel topic for the master turtle to move
        self.cmd_vel_publisher = self.create_publisher(Twist, "/master_turtle/cmd_vel", 10)

        self.control_loop_timer = self.create_timer(0.01, self.control_loop)

        # This subscriber will subscribe to the /alive_turtles topic to retrieve the array of alive turtles
        self.alive_turtles_subscriber = self.create_subscription(TurtleArray, "alive_turtles", self.set_target_turtle, 10)

        # This client will send a CatchTurtle request to the /catch_turtle service when target turtle has been reached
        self.catch_turtle_client = self.create_client(CatchTurtle, "catch_turtle")

    
    def set_pose(self, pose: Pose):
        """
        Callback function when the node subscribes to the master turtle pose topic.
        Updates the node's pose to the master turtle's current pose
        """

        self.pose = pose

    def set_target_turtle(self, alive_turtles : TurtleArray):
        """
        Callback function when subscribed to /alive_turtles topic. 
        Retrieves the array of alive turtles and sets the target turtle to the first turtle in the array
        """

        if len(alive_turtles.turtles) > 0:
            if self.catch_closest_turtle_first:
                closest_turtle = None
                closest_distance = 0.0
                for turtle in alive_turtles.turtles:
                    dist_x = turtle.x - self.pose.x
                    dist_y = turtle.y - self.pose.y
                    distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
                    if distance < closest_distance or closest_turtle == None:
                        closest_turtle = turtle
                        closest_distance = distance
                self.target_turtle = closest_turtle    
            else:
                self.target_turtle = alive_turtles.turtles[0]

    
    def control_loop(self):
        if self.pose == None or self.target_turtle == None:
            return

        # Calculate distance to be travelled using Euclidean distance formula
        dist_x = self.target_turtle.x - self.pose.x
        dist_y = self.target_turtle.y - self.pose.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        cmd = Twist()
        
        if distance > 0.5:
            # Move forward 
            cmd.linear.x = 2 * distance

            # Change orientation of master turtle to face towards the target (rotate around z-axis)
            goal_theta = math.atan2(dist_y, dist_x)

            # Difference between the orientation of master and target turtles: How much the master turtle should turn
            diff = goal_theta - self.pose.theta

            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            
            cmd.angular.z = 6 * diff

        else:
            # Target turtle reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.target_turtle.name)
            self.target_turtle = None

        self.cmd_vel_publisher.publish(cmd)


    def call_catch_turtle_service(self, turtle_name):
        """
        Calls the self.catch_turtle_client when the master turtle has reached the target turtle
        """

        while not self.catch_turtle_client.wait_for_service(10.0):
            self.get_logger().warn("/catch_turtle service is not ready!")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_catch_turtle_service, request=request))


    def callback_catch_turtle_service(self, future, request: CatchTurtle.Request):
        """
        Callback function to check if the target turtle has been killed successfully
        """
        response = future.result()
        if response.success:
            self.get_logger().info(f"{request.name} killed successfully.")
            self.target_turtle = None
        else:
            self.get_logger().info(f"{request.name} killed unsuccessfully.")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()