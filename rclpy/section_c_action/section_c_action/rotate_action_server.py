#!/usr/bin/env python3
import math
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from secion_c_interface.action import Rotate 



def angle_wrap(rad):
    """wrap ให้อยู่ในช่วง [-pi, pi]"""
    return math.atan2(math.sin(rad), math.cos(rad))


def quat_to_yaw(x, y, z, w):
    """คำนวณ yaw จาก quaternion (odom)"""
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


class RotateActionServer(Node):
    def __init__(self):
        super().__init__('rotate_action_server')

        # พารามิเตอร์ P-controller (ปรับได้ตอนรันด้วย ros2 param หรือแก้ค่าด้านล่าง)
        self.declare_parameter('kp', 1.2)           # กำไร P
        self.declare_parameter('max_speed', 1.2)    # rad/s (จำกัดความเร็วหมุน)
        self.declare_parameter('tolerance', 0.02)   # rad (≈1.15°)
        self.declare_parameter('rate_hz', 10.0)     # 10 Hz

        # I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)

        # ตัวแปร odom
        self._have_odom = False
        self._current_yaw = 0.0

        # Action server
        self._server = ActionServer(
            self,
            Rotate,
            'rotate',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb
        )

        self.get_logger().info('rotate_action_server ready.')

    # ---- Odometry callback ----
    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._current_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._have_odom = True

    # ---- Cancel handling ----
    def cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT

    # ---- Core execute ----
    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        kp = float(self.get_parameter('kp').get_parameter_value().double_value)
        max_speed = float(self.get_parameter('max_speed').get_parameter_value().double_value)
        tol = float(self.get_parameter('tolerance').get_parameter_value().double_value)
        rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value)
        dt = 1.0 / rate_hz

        # รอ odom ก่อน (กัน feedback = NaN)
        while rclpy.ok() and not self._have_odom:
            await asyncio.sleep(0.05)

        start_yaw = self._current_yaw
        target_yaw = start_yaw + float(goal.angle)

        feedback = Rotate.Feedback()

        # ใช้ counter ให้มันทนต่อ noise (อยู่ในกรอบ tol ต่อเนื่องสัก 3 รอบ)
        stable_count = 0

        self.get_logger().info(f'Goal angle = {goal.angle:.3f} rad | kp={kp}, max={max_speed}, tol={tol}')

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop_robot()
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled.')
                    return Rotate.Result(success=False)

                # remaining angle = target - current (แบบห่อมุม)
                error = angle_wrap(target_yaw - self._current_yaw)

                # P controller
                omega = kp * error
                # limit speed
                omega = max(-max_speed, min(max_speed, omega))

                # ส่งคำสั่งหมุน
                twist = Twist()
                twist.angular.z = omega
                self.cmd_pub.publish(twist)

                # ส่ง feedback 10 Hz
                feedback.remaining_angle = float(error)
                goal_handle.publish_feedback(feedback)

                # เช็คสำเร็จ
                if abs(error) <= tol:
                    stable_count += 1
                else:
                    stable_count = 0

                if stable_count >= 3:
                    break

                await asyncio.sleep(dt)
        finally:
            # หยุดหุ่นยนต์เสมอเมื่อจบ
            self._stop_robot()

        goal_handle.succeed()
        self.get_logger().info('Rotation finished. Success.')
        result = Rotate.Result()
        result.success = True
        return result

    def _stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)  # ศูนย์หมด = หยุด
        # ส่งซ้ำ 2-3 ครั้งกันพลาด
        self.cmd_pub.publish(msg)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = RotateActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
