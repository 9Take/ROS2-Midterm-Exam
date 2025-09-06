#!/usr/bin/env python3
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from section_c_interface.action import Rotate

class RotateClient(Node):
    def __init__(self):
        super().__init__('rotate_action_client')
        self.client = ActionClient(self, Rotate, 'rotate')

    def send_goal_deg(self, angle_deg: float):
        # แปลง degrees -> radians ก่อนส่งไปที่ server
        angle_rad = math.radians(angle_deg)

        self.get_logger().info("Waiting for action server...")
        self.client.wait_for_server()

        goal = Rotate.Goal()
        goal.angle = float(angle_rad)

        self.get_logger().info(f"Sending goal: {angle_deg:.2f} deg ({angle_rad:.3f} rad)")
        send_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, fb_msg):
        remaining = fb_msg.feedback.remaining_angle
        self.get_logger().info(f"Feedback: remaining {math.degrees(remaining):.1f} deg")

    def result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Result: Goal reached successfully")
        else:
            self.get_logger().warn("Result: Goal aborted")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RotateClient()
    # รับค่าองศาจาก argv (default = 180°)
    deg = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    node.send_goal_deg(deg)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
