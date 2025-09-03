import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cnt = 0
        # Timer to publish velocity every 0.1 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Linear velocity and angular velocity for a circle (radius 0.5 meters)
        self.linear_speed = 0.2  # meters per second
        self.angular_speed = 0.4  # radians per second (approx. for a circle of radius ~0.5 meters)
        
        self.get_logger().info(f"Publishing velocity: Linear Speed = {self.linear_speed} m/s, Angular Speed = {self.angular_speed} rad/s")

    def timer_callback(self):
        self.cnt += 1
        msg = Twist()
        if (self.cnt >= 10):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.destroy_timer(self.timer)
        else:
            # Set linear velocity (forward speed)
            msg.linear.x = self.linear_speed
            
            # Set angular velocity (turning speed)
            msg.angular.z = self.angular_speed

        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the message being published
        self.get_logger().info(f'Publishing: Linear Speed = {msg.linear.x}, Angular Speed = {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    circle_publisher = CirclePublisher()
    rclpy.spin(circle_publisher)
    circle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
