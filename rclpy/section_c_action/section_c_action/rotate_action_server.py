#!/usr/bin/env python3
import math
import time  # <<<<<<<<<<<<<<<<<<  เพิ่มอันนี้

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from section_c_interface.action import Rotate

def normalize_angle(a: float) -> float:
    a = (a + math.pi) % (2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi

class RotateActionServer(Node):
    def __init__(self):
        super().__init__('rotate_action_server')
        # params
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('max_w', 1.2)
        self.declare_parameter('min_w', 0.15)
        # self.declare_parameter('kp', 0.5)
        # self.declare_parameter('max_w', 3)
        # self.declare_parameter('min_w', 0.15)
        self.declare_parameter('tolerance_deg', 5.0)
        self.declare_parameter('odom_topic', '/odom')

        # pubs/subs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._x = self._y = self._yaw = 0.0
        self._have_odom = False
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 25)

        # action
        self._server = ActionServer(
            self, Rotate, 'rotate',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )
        self.get_logger().info('rotate_action_server ready')

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)
        self._have_odom = True

    def goal_cb(self, goal_request):
        self.get_logger().info(f"Goal received: angle={goal_request.angle:.3f} rad")
        return GoalResponse.ACCEPT

    def cancel_cb(self, _):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        # wait first odom
        while rclpy.ok() and not self._have_odom:
            rclpy.spin_once(self, timeout_sec=0.1)

        target_delta = float(goal_handle.request.angle)
        kp   = self.get_parameter('kp').value
        maxw = self.get_parameter('max_w').value
        minw = self.get_parameter('min_w').value
        tol  = math.radians(self.get_parameter('tolerance_deg').value)

        start_yaw  = self._yaw
        target_yaw = normalize_angle(start_yaw + target_delta)

        fb = Rotate.Feedback()
        cmd = Twist()
        dt = 0.1
        stable_hits = 0
        needed_hits = 5

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                res = Rotate.Result(); res.success = False
                self.get_logger().info("Goal canceled")
                return res

            rclpy.spin_once(self, timeout_sec=0.0)

            remain = normalize_angle(target_yaw - self._yaw)
            fb.remaining_angle = remain
            goal_handle.publish_feedback(fb)

            self.get_logger().info(
                f"odom -> x={self._x:.3f}, y={self._y:.3f}, yaw={math.degrees(self._yaw):.1f}° "
                f"(remain={math.degrees(remain):.1f}°)"
            )

            if abs(remain) <= tol:
                stable_hits += 1
                if stable_hits >= needed_hits:
                    break
            else:
                stable_hits = 0

            w = kp * remain
            w = max(-maxw, min(maxw, w))
            if abs(w) < minw:
                w = math.copysign(minw, w)

            cmd.linear.x = 0.0
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)

            time.sleep(dt)  # <<<<<<<<<<<<<<<<<<  ใช้ time.sleep() แทน rclpy.sleep()

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
