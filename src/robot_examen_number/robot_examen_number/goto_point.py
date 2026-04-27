#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, Pose2D


class GotoPoint(Node):

    def __init__(self):
        super().__init__('goto_point')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('goal_x',          0.0)
        self.declare_parameter('goal_y',          0.0)
        self.declare_parameter('goal_theta',       0.0)   # orientación final deseada (rad)
        self.declare_parameter('kp_linear',        0.0)   # ganancia velocidad lineal
        self.declare_parameter('kp_angular',       0.0)   # ganancia velocidad angular
        self.declare_parameter('dist_tolerance',   0.05)  # tolerancia de posición (m)
        self.declare_parameter('angle_tolerance',  0.05)  # tolerancia de orientación (rad)
        self.declare_parameter('max_linear_vel',   0.4)   # m/s máximo
        self.declare_parameter('max_angular_vel',  1.2)   # rad/s máximo

        self.gx      = self.get_parameter('goal_x').value
        self.gy      = self.get_parameter('goal_y').value
        self.g_th    = math.radians(self.get_parameter('goal_theta').value)
        self.kp_v    = self.get_parameter('kp_linear').value
        self.kp_w    = self.get_parameter('kp_angular').value
        self.tol_d   = self.get_parameter('dist_tolerance').value
        self.tol_a   = self.get_parameter('angle_tolerance').value
        self.vmax    = self.get_parameter('max_linear_vel').value
        self.wmax    = self.get_parameter('max_angular_vel').value

        # Estado
        self.pose      = Pose2D()
        self.reached   = False

        # Pub / Sub
        self.create_subscription(Pose2D, '/robot_pose', self._cb_pose, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self._control_loop)   # 20 Hz

        self.get_logger().info(
            f'GotoPoint OMNI iniciado — '
            f'objetivo: ({self.gx:.2f}, {self.gy:.2f}) θ={math.degrees(self.g_th):.1f}°'
        )

    def _cb_pose(self, msg: Pose2D):
        self.pose = msg

    def _clamp(self, v, limit):
        return max(-limit, min(limit, v))

    def _control_loop(self):
        if self.reached:
            self.pub_cmd.publish(Twist())   # detener
            return

        x, y, th = self.pose.x, self.pose.y, self.pose.theta

        # ── Error de posición en frame MUNDO ──────────────────────────────
        dx_world = self.gx - x
        dy_world = self.gy - y
        dist = math.hypot(dx_world, dy_world)

        # ── Rotar error al frame ROBOT ─────────────────────────────────────
        #    El robot omni necesita vx/vy en su propio frame
        cos_th = math.cos(th)
        sin_th = math.sin(th)
        vx_robot = ######
        vy_robot = ######

        # ── Error de orientación ───────────────────────────────────────────
        e_th = self.g_th - th
        e_th = math.atan2(math.sin(e_th), math.cos(e_th))   

        cmd = Twist()

        if dist < self.tol_d and abs(e_th) < self.tol_a:
            # ── Llegó al objetivo con orientación correcta ─────────────────
            self.reached = True
            self.get_logger().info(
                f'Objetivo alcanzado: x={x:.3f} y={y:.3f} θ={math.degrees(th):.1f}°'
            )
            return

        if dist >= self.tol_d:
           
            cmd.linear.x = #####
            cmd.linear.y = #####

        cmd.angular.z = self._clamp(self.kp_w * e_th, self.wmax)

        self.pub_cmd.publish(cmd)

        self.get_logger().info(
            f'd={dist:.2f}m  vx={cmd.linear.x:.2f}  vy={cmd.linear.y:.2f}  '
            f'ω={cmd.angular.z:.2f}  θ_err={math.degrees(e_th):.1f}°',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = GotoPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
