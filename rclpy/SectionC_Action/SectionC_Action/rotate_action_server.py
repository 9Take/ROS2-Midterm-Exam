import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RotateActionServer(Node):
    def __init__(self):
        super().__init__('rotate_action_server')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
    
        # Timer to publish velocity every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)

    
     def timer_callback(self):
        msg = Twist()
        
        
        # Set angular velocity (turning speed)
        msg.angular.z = self._______ #calculate from P control Kp
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the message being published
        self.get_logger().info(f'Angular Speed = {msg.angular.z}')

    
    def main(args=None):
    rclpy.init(args=args)
    rotate_action_server = RotateActionServer()
    rclpy.spin(rotate_action_server)
    rotate_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
