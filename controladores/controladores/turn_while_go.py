#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math ,time


class TurnWhileGoController(Node):
    def __init__(self):
        super().__init__('turn_while_go_controller')

        # Lista de metas (x, y)
        self.goals = [(7.5, 5.5),(7.5, 7.5),(5.5, 7.5),(5.5, 5.5)]

        #self.goals = [(7.5, 5.5),(5.5, 3.5),(7.5, 1.5),(3.5, 2.5),(0.0, 1.5),(1.5, 3.5),(0.0, 5.5),(1.5, 5.5),(3.5, 7.5),(5.5, 5.5)]
        self.current_goal_index = 0

        # Tolerancias
        self.distance_tolerance = 0.05

        # Pose actual
        self.pose = None

        # Tiempo de inicio para medir duración
        self.start_time = None

        # Publicador y suscriptor
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

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

        distance = math.hypot(dx, dy)
        desired_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_theta - self.pose.theta)
        if self.start_time is None:
            self.start_time = time.time()


        twist = Twist()

        if distance > self.distance_tolerance:
            # Control proporcional simultáneo
            
            twist.linear.x = max(min(distance*1.5, 0.2), -0.2)
            twist.angular.z = max(-0.6, min(4.0 * angle_error, 0.6))  # Limitar angular
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            elapsed_time = time.time() - self.start_time
            final_error = distance

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
    node = TurnWhileGoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
