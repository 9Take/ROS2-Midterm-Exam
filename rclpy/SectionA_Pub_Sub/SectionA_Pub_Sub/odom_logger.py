import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        
        # Subscriber to the /odom topic (Odometry message)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        # Extract position (x, y)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract quaternion orientation
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # Log the position and yaw (heading) angle
        self.get_logger().info(f'Position: x={x:.2f}, y={y:.2f}, Yaw: {yaw:.2f} radians')

def main(args=None):
    rclpy.init(args=args)
    odom_logger = OdomLogger()
    rclpy.spin(odom_logger)
    odom_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
