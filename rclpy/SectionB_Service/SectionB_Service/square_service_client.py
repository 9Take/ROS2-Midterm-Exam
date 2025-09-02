import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class SquareServiceClient(Node):
    def __init__(self):
        super().__init__('square_service_client')

        # Create the client for 'square_service'
        self.cli = self.create_client(Empty, 'square_service')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = Empty.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = SquareServiceClient()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().error(f'Service call failed: {e}')
            else:
                node.get_logger().info('Square movement completed!')
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
