import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import time
import math

class SquareServiceServer(Node):
    def __init__(self):
        super().__init__('square_service_server')

        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create the service server
        self.srv = self.create_service(Empty, 'square_service', self.square_callback)

        self.get_logger().info("Square service server ready...")

    def square_callback(self, request, response):
        self.get_logger().info("Service called: Moving in a square!")

        # Create Twist message
        move = Twist()

        # Define square parameters
        side_length = 0.5  # meters
        linear_speed = 0.1  # m/s
        angular_speed = math.pi / 4  # rad/s, 90 degree turn

        # Time to move forward for one side
        move_time = side_length / linear_speed

        # Time to rotate 90 degrees
        turn_time = (math.pi / 2) / angular_speed

        # Move in square
        for i in range(4):
            # Move forward
            move.linear.x = linear_speed
            move.angular.z = 0.0
            self.publish_velocity(move, move_time)

            # Rotate 90 degrees
            move.linear.x = 0.0
            move.angular.z = angular_speed
            self.publish_velocity(move, turn_time)

        # Stop at the end
        move.linear.x = 0.0
        move.angular.z = 0.0
        self.publisher.publish(move)

        self.get_logger().info("Finished moving in square.")

        return response

    def publish_velocity(self, move_msg, duration):
        """Helper function to continuously publish velocity for a duration"""
        # start_time = self.get_clock().now().to_sec()
        # while (self.get_clock().now().to_sec() - start_time) < duration:
        #     self.publisher.publish(move_msg)
        #     rclpy.spin_once(self, timeout_sec=0.1)
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duration:
            self.publisher.publish(move_msg)
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = SquareServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
