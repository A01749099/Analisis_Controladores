#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math, time


class LyapunovController(Node):
    def __init__(self):
        super().__init__('lyapunov_controller')

        # Lista de metas (x, y)
        self.goals = [(7.5, 5.5),(7.5, 7.5),(5.5, 7.5),(5.5, 5.5)]

        #self.goals = [(7.5, 5.5),(5.5, 3.5),(7.5, 1.5),(3.5, 2.5),(0.0, 1.5),(1.5, 3.5),(0.0, 5.5),(1.5, 5.5),(3.5, 7.5),(5.5, 5.5)]
        self.current_goal_index = 0
        self.goal_tolerance = 0.05

        # Tiempo de inicio para medir duración
        self.start_time = None
        # Parámetros del controlador Lyapunov
        self.v_ri = 0.2     # Velocidad de referencia
        self.k1 = 1
        self.k2 = 0.1

        self.pose = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None or self.current_goal_index >= len(self.goals):
            return

        goal_x, goal_y = self.goals[self.current_goal_index]
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        if self.start_time is None:
            self.start_time = time.time()

        #distance = math.hypot(dx, dy)

        l = math.hypot(dx, dy)
        if l < self.goal_tolerance:
            elapsed_time = time.time() - self.start_time
            final_error = l

            self.get_logger().info(f"[{self.current_goal_index+1}] Meta alcanzada")
            self.get_logger().info(f" Tiempo: {elapsed_time:.2f} s |  Error final: {final_error:.4f} m")
            self.start_time = None
            self.get_logger().info(f"[{self.current_goal_index+1}] Meta alcanzada")
            self.current_goal_index += 1
            if self.current_goal_index >= len(self.goals):
                self.get_logger().info("✅ Todas las metas alcanzadas.")
                self.cmd_vel_pub.publish(Twist())
                self.timer.cancel()
            return

        # Ángulo objetivo
        theta_G = math.atan2(dy, dx)
        theta = self.pose.theta

        # Errores de orientación
        theta_er = self.normalize_angle(theta_G - theta)

        # Cálculo de velocidades usando la función de Lyapunov
        v = self.v_ri * math.cos(theta_er)
        if abs(theta_er) > 1e-6:
            omega = self.k1 * theta_er + \
                    (self.v_ri * math.cos(theta_er) * math.sin(theta_er) * (theta_er + self.k2 * theta_G)) / theta_er
        else:
            omega = self.k1 * theta_er  # Aproximación cuando theta_er es muy pequeño

        omega = max(min(omega, 0.6), -0.6)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        """Normaliza el ángulo entre -pi y pi"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = LyapunovController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
