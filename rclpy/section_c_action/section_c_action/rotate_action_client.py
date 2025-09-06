import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from section_c_interface.action import Rotate


class RotateClient(Node):
    def __init__(self):
        super().__init__('rotate_action_client')
        self._ac = ActionClient(self, Rotate, 'rotate')

    def send_goal(self, angle_rad: float):
        self.get_logger().info(f"Waiting for action server...")
        self._ac.wait_for_server()

        goal = Rotate.Goal()
        goal.angle = float(angle_rad)

        self.get_logger().info(f"Sending goal: {goal.angle:.3f} rad")
        send_future = self._ac.send_goal_async(
            goal, feedback_callback=self._fb_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    # callbacks
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_cb)

    def _fb_cb(self, fb_msg):
        remaining = fb_msg.feedback.remaining_angle
        self.get_logger().info(f"Feedback: remaining {remaining:.3f} rad")

    def _result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Result: Goal reached successfully")
        else:
            self.get_logger().warn("Result: Goal aborted")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RotateClient()
    # อ่านมุมจาก argv (rad). default = +pi/2
    angle = float(sys.argv[1]) if len(sys.argv) > 1 else (math.pi / 2.0)
    node.send_goal(angle)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
