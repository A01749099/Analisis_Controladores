#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from turtlesim.srv import SetPen, TeleportAbsolute


class TurnThenGoController(Node):
    def __init__(self):
        super().__init__('turn_then_go_controller')

        self.pose = None
        self.painter_pose = None

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
        self.painter_pose_sub = self.create_subscription(Pose, '/painter/pose', self.painter_pose_callback, 10)
        # Clientes de servicio
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.set_pen_client_painter = self.create_client(SetPen, '/painter/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/painter/teleport_absolute')

        # Esperar servicios necesarios
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /turtle1/set_pen...')

        # Activar lápiz rojo para turtle1
        self.set_pen(255, 0, 0, 2, 0)

        # Generar trayectoria circular del painter
        self.curva_s()

        # Mover painter al punto inicial sin pintar
        self.teleport_client.wait_for_service()
        initial = self.painter_points[0]
        req_teleport = TeleportAbsolute.Request()
        req_teleport.x, req_teleport.y, req_teleport.theta = initial
        self.teleport_client.call_async(req_teleport)

        # Activar lápiz verde para painter
        pen_req = SetPen.Request()
        pen_req.r = 0
        pen_req.g = 255
        pen_req.b = 0
        pen_req.width = 2
        pen_req.off = 0
        self.set_pen_client_painter.call_async(pen_req)

        self.painter_index = 0
        self.painter_delay_counter = 0
        self.painter_delay_interval = 6.3

        self.total_distance_error = 0.0
        self.error_count = 0
        self.finished = False
        self.max_distance_error = 0.0
        self.previous_dx = None
        self.oscillations = 0


        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)


    def circulo(self):
        self.painter_points = []
        temp_points = []
        center_x, center_y = 3.54, 5.54
        radius = 2.0
        num_points = 100

        for i in range(num_points + 1):
            angle = (2 * math.pi * i) / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            theta = angle
            self.painter_points.append((x, y, theta))

    def curva_s(self):
        self.painter_points = []
        temp_points = []
        # Longitud máxima horizontal y amplitud vertical que caben en la zona
        length = 4.0
        amplitude = 2.0

        for i in range(100):
            t = i / 99
            x = t * length
            y = amplitude * math.sin(2 * math.pi * t)
            theta = math.atan2(2 * math.pi * amplitude * math.cos(2 * math.pi * t), length)
            temp_points.append((x, y, theta))

        # Mover para que inicie en (5.4, 5.4)
        offset_x = 5.4 - temp_points[0][0]
        offset_y = 5.4 - temp_points[0][1]

        for x, y, theta in temp_points:
            x_new = x + offset_x
            y_new = y + offset_y
            if 1 <= x_new <= 10 and 1 <= y_new <= 10:
                self.painter_points.append((x_new, y_new, theta))


    def estrella(self):
        self.painter_points = []
        temp_points = []

        num_points = 5
        inner_radius = 1.0
        outer_radius = 2.2
        center_x, center_y = 0, 0

        # Generar vértices de la estrella
        for i in range(2 * num_points):
            angle = i * math.pi / num_points
            r = outer_radius if i % 2 == 0 else inner_radius
            x = center_x + r * math.cos(angle)
            y = center_y + r * math.sin(angle)
            theta = angle
            temp_points.append((x, y, theta))

        # Desplazar para que el primer punto sea (5.4, 5.4)
        offset_x = 5.4 - temp_points[0][0]
        offset_y = 5.4 - temp_points[0][1]

        # Interpolar puntos entre cada par de vértices
        steps_between = 10  # número de puntos entre vértices

        for i in range(len(temp_points)):
            x0, y0, _ = temp_points[i]
            x1, y1, _ = temp_points[(i + 1) % len(temp_points)]

            x0 += offset_x
            y0 += offset_y
            x1 += offset_x
            y1 += offset_y

            for step in range(steps_between + 1):
                t = step / steps_between
                x_interp = (1 - t) * x0 + t * x1
                y_interp = (1 - t) * y0 + t * y1
                theta = math.atan2(y1 - y0, x1 - x0)
                
                if 1 <= x_interp <= 10 and 1 <= y_interp <= 10:
                    self.painter_points.append((x_interp, y_interp, theta))


    def espiral(self):
        self.painter_points = []
        temp_points = []

        num_turns = 1.5
        num_points = 100
        a = 0.5  # crecimiento más lento

        for i in range(num_points):
            angle = i * (2 * math.pi) / (num_points / num_turns)
            radius = a * angle
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            theta = angle
            temp_points.append((x, y, theta))

        # Mover para que comience en (5.4, 5.4)
        offset_x = 5.4 - temp_points[0][0]
        offset_y = 5.4 - temp_points[0][1]

        for x, y, theta in temp_points:
            x_new = x + offset_x
            y_new = y + offset_y
            if 1 <= x_new <= 10 and 1 <= y_new <= 10:
                self.painter_points.append((x_new, y_new, theta))


    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.set_pen_client.call_async(req)

    def painter_pose_callback(self, msg):
        self.painter_pose = msg

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None or self.painter_pose is None:
            return

        #goal_x, goal_y = self.goals[self.current_goal_index]

        # Mover painter paso a paso
        if self.painter_index < len(self.painter_points):
            self.painter_delay_counter += 1
            if self.painter_delay_counter >= self.painter_delay_interval:
                x, y, theta = self.painter_points[self.painter_index]
                req = TeleportAbsolute.Request()
                req.x, req.y, req.theta = x, y, theta
                self.teleport_client.call_async(req)
                self.painter_index += 1
                self.painter_delay_counter = 0

        dx = self.painter_pose.x - self.pose.x
        dy = self.painter_pose.y - self.pose.y
        desired_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_theta - self.pose.theta)
        distance = math.hypot(dx, dy)

        self.total_distance_error += distance
        self.error_count += 1
        self.max_distance_error = max(self.max_distance_error, distance)
        if self.previous_dx is not None and (dx * self.previous_dx < 0):
            self.oscillations += 1
        self.previous_dx = dx

        

        # Solo imprimir al terminar la trayectoria
        if self.painter_index >= len(self.painter_points) and not self.finished:
            avg_error = self.total_distance_error / self.error_count
            self.get_logger().info(f"[FINAL] Avg Distance Error: {avg_error:.4f}")
            
            self.get_logger().info(f"Max Error: {self.max_distance_error:.3f}")
            self.get_logger().info(f"Oscillations: {self.oscillations}")
            
            self.finished = True  # ← evita que lo vuelva a imprimir

            #  Detener turtle1
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            
            self.finished = True
            self.get_logger().info(" Trayectoria finalizada. Turtle1 detenido.")
            
            self.timer.cancel()  # Cancelar bucle
            return

        
        twist = Twist()

        if self.phase == "turn":
            if abs(angle_error) > self.angle_tolerance:
                twist.angular.z = 1.5 * angle_error
                
            else:
                twist.angular.z = 0.0
                self.phase = "go"
                #self.start_time = time.time()
                self.get_logger().info(f"[{self.current_goal_index+1}] Fase cambiada: go")

        if self.phase == "turn":
            if abs(angle_error) > self.angle_tolerance:
                twist.angular.z = 3.5 * angle_error
                twist.linear.x = 0.0  # No avanza mientras gira
            else:
                twist.angular.z = 0.0
                self.phase = "go"
                self.get_logger().info(f"[{self.current_goal_index+1}] Fase cambiada: go")

        elif self.phase == "go":
            # 🔁 CORRECCIÓN: Si se desvía mucho, vuelve a girar
            if abs(angle_error) > 0.2:  # Tolerancia de desviación angular
                self.get_logger().info(f"[{self.current_goal_index+1}] Desviación detectada, regresando a fase: turn")
                self.phase = "turn"
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif distance > self.distance_tolerance:
                twist.linear.x = min(1.0 * distance, 0.2)
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

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
