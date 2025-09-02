import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('square_service_server')
        self.publisher_ = self.create_publisher(Empty, 'cmd_vel', 10)