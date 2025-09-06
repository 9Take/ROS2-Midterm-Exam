#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from section_c_interface.action import Rotate   # uses Rotate.action


def normalize_angle(a: float) -> float:
    a = (a + math.pi) % (2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi


class RotateActionServer(Node):
    def __init__(self):
        super().__init__('rotate_action_server')

        # pubs/subs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._yaw = 0.0
        self.create_subscription(Odometry, '/odom', self._odom_cb, 25)

        # parameters (tune as needed)
        self.declare_parameter('kp', 1.2)            # proportional gain
        self.declare_parameter('max_w', 1.2)         # max angular speed (rad/s)
        self.declare_parameter('min_w', 0.15)        # minimum to overcome stiction
        self.declare_parameter('tolerance_deg', 10.) # success tolerance

        # action server
        self._server = ActionServer(
            self, Rotate, 'rotate',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )
        self.get_logger().info('rotate_action_server ready')

    # --- callbacks ---
    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw from quaternion (atan2 formula)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_cb(self, goal_request):
        self.get_logger().info(f"Goal received: angle={goal_request.angle:.3f} rad")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_cb(self, _):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    # --- execution ---
    def execute_cb(self, goal_handle):
        angle = float(goal_handle.request.angle)
        kp = self.get_parameter('kp').value
        max_w = self.get_parameter('max_w').value
        min_w = self.get_parameter('min_w').value
        tol = math.radians(self.get_parameter('tolerance_deg').value)

        start_yaw = self._yaw
        target_yaw = normalize_angle(start_yaw + angle)

        twist = Twist()
        fb = Rotate.Feedback()

        # 10 Hz loop
        dt = 0.1
        stable_hits = 0
        required_hits = 5  # within tolerance for ~0.5 s

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                res = Rotate.Result(); res.success = False
                self.get_logger().info("Goal canceled")
                return res

            remaining = normalize_angle(target_yaw - self._yaw)
            fb.remaining_angle = remaining
            goal_handle.publish_feedback(fb)

            if abs(remaining) <= tol:
                stable_hits += 1
                if stable_hits >= required_hits:
                    break
            else:
                stable_hits = 0

            w = kp * remaining
            w = max(-max_w, min(max_w, w))
            if abs(w) < min_w:
                w = math.copysign(min_w, w)

            twist.linear.x = 0.0
            twist.angular.z = w
            self.cmd_pub.publish(twist)
            time.sleep(dt)

        self._stop()
        goal_handle.succeed()
        res = Rotate.Result(); res.success = True
        self.get_logger().info("Goal reached successfully")
        return res

    def _stop(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = RotateActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
