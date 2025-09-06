import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from section_c_interface.action import Rotate  # from interface package


def normalize_angle(angle):
    """wrap to [-pi, pi]"""
    a = (angle + math.pi) % (2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi


class RotateActionServer(Node):
    def __init__(self):
        super().__init__('rotate_action_server')

        # pub /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # subscribe /odom to read yaw
        self._yaw = 0.0
        self.create_subscription(Odometry, '/odom', self._odom_cb, 25)

        # action server
        self._server = ActionServer(
            self,
            Rotate,
            'rotate',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            handle_accepted_callback=None,
            cancel_callback=self.cancel_cb)

        # params for P controller
        self.declare_parameter('kp', 1.2)          # tune this
        self.declare_parameter('max_w', 1.2)       # rad/s
        self.declare_parameter('min_w', 0.15)      # rad/s to overcome static
        self.declare_parameter('tolerance_deg', 10.0)

        self.get_logger().info('rotate_action_server ready')

    # --- callbacks ---
    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw from quaternion
        # compute yaw analytically
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_cb(self, goal_req):
        # accept every goal
        self.get_logger().info(f"Received goal: angle={goal_req.angle:.3f} rad")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    # --- main execute ---
    def execute_cb(self, goal_handle):
        angle_target = float(goal_handle.request.angle)
        kp = self.get_parameter('kp').value
        max_w = self.get_parameter('max_w').value
        min_w = self.get_parameter('min_w').value
        tol_rad = math.radians(self.get_parameter('tolerance_deg').value)

        # latch starting yaw and target yaw
        start_yaw = self._yaw
        target_yaw = normalize_angle(start_yaw + angle_target)

        fb = Rotate.Feedback()
        twist = Twist()

        # 10 Hz loop
        dt = 0.1
        last_ok_hits = 0
        required_hits = 5  # stay in tolerance for ~0.5 s

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                # stop
                self._stop_robot()
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                result = Rotate.Result()
                result.success = False
                return result

            # remaining angle (shortest path)
            remaining = normalize_angle(target_yaw - self._yaw)
            fb.remaining_angle = remaining
            goal_handle.publish_feedback(fb)

            # check done (tolerance ±10 deg)
            if abs(remaining) <= tol_rad:
                last_ok_hits += 1
            else:
                last_ok_hits = 0

            if last_ok_hits >= required_hits:
                break

            # P controller on yaw error
            w = kp * remaining
            # saturate
            w = max(-max_w, min(max_w, w))
            # apply a minimum magnitude to overcome stiction
            if abs(w) < min_w:
                w = math.copysign(min_w, w)

            twist.linear.x = 0.0
            twist.angular.z = w
            self.cmd_pub.publish(twist)

            time.sleep(dt)

        # stop & succeed
        self._stop_robot()
        goal_handle.succeed()
        res = Rotate.Result()
        res.success = True
        self.get_logger().info("Goal reached successfully")
        return res

    def _stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RotateActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
