#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class TurnThenGoController(Node):
    def __init__(self):
        super().__init__('turn_then_go_controller')

        # Lista de metas (x, y)
        self.goals = [(7.5, 5.5),(7.5, 7.5),(5.5, 7.5),(5.5, 5.5)]

        self.goals = [(7.5, 5.5),(5.5, 3.5),(7.5, 1.5),(3.5, 2.5),(0.0, 1.5),(1.5, 3.5),(0.0, 5.5),(1.5, 5.5),(3.5, 7.5),(5.5, 5.5)]
        self.current_goal_index = 0

        # Tolerancias
        self.angle_tolerance = 0.01
        self.distance_tolerance = 0.05

        # Fase inicial
        self.phase = "turn"

        # Pose actual
        self.pose = None

        # Publicador y suscriptor
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Tiempo de inicio para medir duración
        self.start_time = None

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None or self.current_goal_index >= len(self.goals):
            return

        goal_x, goal_y = self.goals[self.current_goal_index]
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y

        desired_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_theta - self.pose.theta)
        distance = math.hypot(dx, dy)
        if self.start_time is None:
            self.start_time = time.time()

        twist = Twist()

        if self.phase == "turn":
            if abs(angle_error) > self.angle_tolerance:
                twist.angular.z = 1.5 * angle_error
                
            else:
                twist.angular.z = 0.0
                self.phase = "go"
                #self.start_time = time.time()
                self.get_logger().info(f"[{self.current_goal_index+1}] Fase cambiada: go")

        elif self.phase == "go":
            if distance > self.distance_tolerance:
                twist.linear.x = min(1.0 * distance, 0.2)  # Límite de velocidad
            else:
                twist.linear.x = 0.0
                elapsed_time = time.time() - self.start_time
                final_error = distance

                self.get_logger().info(f"[{self.current_goal_index+1}] Meta alcanzada")
                self.get_logger().info(f"⏱️ Tiempo: {elapsed_time:.2f} s | 📏 Error final: {final_error:.4f} m")
                self.start_time = None

                self.get_logger().info(f"[{self.current_goal_index+1}] Meta alcanzada")
                self.phase = "turn"
                self.current_goal_index += 1

                if self.current_goal_index >= len(self.goals):
                    self.get_logger().info("✅ Todas las metas alcanzadas.")
                    
                    # Detener movimiento
                    stop_msg = Twist()
                    self.cmd_vel_pub.publish(stop_msg)

                    self.timer.cancel()
                    return

        twist.angular.z = max(min(twist.angular.z, 0.6), -0.6)
        twist.linear.x = max(min(twist.linear.x, 0.2), -0.2)
        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        """Normaliza ángulo entre -pi y pi"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TurnThenGoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
