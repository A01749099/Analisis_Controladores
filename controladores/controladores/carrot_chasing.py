#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math,time


class CarrotChasingController(Node):
    def __init__(self):
        super().__init__('carrot_chasing_controller')

        # Lista de metas (x, y)
        self.goals = [(7.5, 5.5),(7.5, 7.5),(5.5, 7.5),(5.5, 5.5)]

        #self.goals = [(7.5, 5.5),(5.5, 3.5),(7.5, 1.5),(3.5, 2.5),(0.0, 1.5),(1.5, 3.5),(0.0, 5.5),(1.5, 5.5),(3.5, 7.5),(5.5, 5.5)]
        self.current_goal_index = 0

        # Distancia a la "zanahoria"
        self.lookahead_distance = 1.0

        # Tolerancia para considerar una meta alcanzada
        self.goal_tolerance = 0.05

        # Estado actual
        self.pose = None

        # Tiempo de inicio para medir duración
        self.start_time = None

        # ROS2 publishers/subscribers
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
        distance_to_goal = math.hypot(dx, dy)

        if self.start_time is None:
            self.start_time = time.time()


        # Avanzar al siguiente objetivo si ya se llegó
        if distance_to_goal < self.goal_tolerance:

            elapsed_time = time.time() - self.start_time
            final_error = distance_to_goal

            self.get_logger().info(f"[{self.current_goal_index+1}] Meta alcanzada")
            self.get_logger().info(f" Tiempo: {elapsed_time:.2f} s |  Error final: {final_error:.4f} m")
            self.start_time = None
            self.get_logger().info(f"[{self.current_goal_index+1}] Meta alcanzada")
            #print(self.pose.x)
            #print(self.pose.y)
            self.current_goal_index += 1

            if self.current_goal_index >= len(self.goals):
                self.get_logger().info("✅ Todas las metas alcanzadas.")
                print(self.pose.x)
                print(self.pose.y)
                self.cmd_vel_pub.publish(Twist())
                self.timer.cancel()
            return

        # Dirección hacia la zanahoria
        path_theta = math.atan2(dy, dx)

        # Punto de zanahoria
        carrot_x = self.pose.x + self.lookahead_distance * math.cos(path_theta)
        carrot_y = self.pose.y + self.lookahead_distance * math.sin(path_theta)

        # Ángulo hacia la zanahoria
        carrot_dx = carrot_x - self.pose.x
        carrot_dy = carrot_y - self.pose.y
        carrot_theta = math.atan2(carrot_dy, carrot_dx)
        angle_error = self.normalize_angle(carrot_theta - self.pose.theta)

        # Control proporcional
        twist = Twist()
        twist.linear.x = 0.2  # constante
        twist.angular.z = 1.0 * angle_error
        twist.angular.z = max(min(twist.angular.z, 0.6), -0.6)

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
    node = CarrotChasingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 