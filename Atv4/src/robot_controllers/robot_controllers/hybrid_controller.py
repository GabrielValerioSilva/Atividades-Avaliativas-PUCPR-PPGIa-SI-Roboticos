import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def trimf(x: float, a: float, b: float, c: float) -> float:
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    if x < b:
        return (x - a) / (b - a) if b > a else 1.0
    return (c - x) / (c - b) if c > b else 1.0


def trapmf(x: float, a: float, b: float, c: float, d: float) -> float:
    if x <= a or x >= d:
        return 0.0
    if b <= x <= c:
        return 1.0
    if x < b:
        return (x - a) / (b - a) if b > a else 1.0
    return (d - x) / (d - c) if d > c else 1.0


class HybridPidFuzzyController(Node):
    def __init__(self):
        super().__init__('hybrid_pid_fuzzy_controller')

        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 0.5)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data
        )

        # Estado de odometria e LiDAR
        self.x = None
        self.y = None
        self.yaw = 0.0
        self.front_dist = 3.5
        self.left_dist = 3.5
        self.right_dist = 3.5
        self.goal_reached = False

        # Limiares calibrados para não disparar cedo nem travar em giro
        self.enter_fuzzy_dist = 0.42   # Só entra em Fuzzy abaixo de 0.42 m
        self.exit_fuzzy_dist = 0.52    # Volta ao PID acima de 0.52 m
        self.mode = 'PID'
        self.avoid_dir = 1.0
        self.fuzzy_hold_ticks = 0      # Tempo mínimo contornando o pilar

        # Ganhos PID
        self.kp_lin, self.ki_lin, self.kd_lin = 0.50, 0.02, 0.10
        self.kp_ang, self.ki_ang, self.kd_ang = 1.50, 0.05, 0.20
        self.int_dist = 0.0
        self.prev_dist = 0.0
        self.int_ang = 0.0
        self.prev_ang = 0.0
        self.dt = 0.1
        self.tick = 0

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'Controlador HÍBRIDO (PID + Fuzzy) iniciado! Destino: ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def scan_cb(self, msg: LaserScan):
        front, left, right = [], [], []
        for i, d in enumerate(msg.ranges):
            if math.isinf(d) or math.isnan(d) or d < msg.range_min or d > msg.range_max:
                continue
            a = math.atan2(
                math.sin(msg.angle_min + i * msg.angle_increment),
                math.cos(msg.angle_min + i * msg.angle_increment)
            )
            # Corredor frontal estreito (±20°) para evitar falsos positivos laterais
            if -0.35 <= a <= 0.35:
                front.append(d)
            elif 0.35 < a <= 1.40:
                left.append(d)
            elif -1.40 <= a < -0.35:
                right.append(d)

        self.front_dist = min(front) if front else msg.range_max
        self.left_dist = (sum(left) / len(left)) if left else msg.range_max
        self.right_dist = (sum(right) / len(right)) if right else msg.range_max

    def pub_cmd(self, vx: float, wz: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(vx)
        cmd.twist.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    def compute_pid_position(self, dist_err: float, ang_err: float):
        self.int_dist = max(-0.5, min(0.5, self.int_dist + dist_err * self.dt))
        der_dist = (dist_err - self.prev_dist) / self.dt
        self.prev_dist = dist_err

        self.int_ang = max(-0.5, min(0.5, self.int_ang + ang_err * self.dt))
        der_ang = (ang_err - self.prev_ang) / self.dt
        self.prev_ang = ang_err

        w = self.kp_ang * ang_err + self.ki_ang * self.int_ang + self.kd_ang * der_ang
        w = max(-0.80, min(0.80, w))

        if abs(ang_err) < 0.45:
            v = self.kp_lin * dist_err + self.ki_lin * self.int_dist + self.kd_lin * der_dist
            v = max(0.0, min(0.16, v))
        else:
            v = 0.0
        return v, w

    def compute_fuzzy_avoidance(self):
        # Fuzzificação da distância frontal (m)
        f_critico = trapmf(self.front_dist, 0.0, 0.0, 0.22, 0.32)
        f_perto = trimf(self.front_dist, 0.25, 0.36, 0.48)
        f_livre = trapmf(self.front_dist, 0.42, 0.55, 4.0, 4.5)

        # Mantém avanço contínuo durante o contorno para não girar parado no lugar
        v_rules = [
            (f_critico, 0.02),
            (f_perto, 0.08),
            (f_livre, 0.13),
        ]
        w_rules = [
            (f_critico, 0.75 * self.avoid_dir),
            (f_perto, 0.55 * self.avoid_dir),
            (f_livre, 0.20 * self.avoid_dir),
        ]

        v_den = sum(mu for mu, _ in v_rules)
        w_den = sum(mu for mu, _ in w_rules)
        v = sum(mu * val for mu, val in v_rules) / v_den if v_den > 0.0 else 0.08
        w = sum(mu * val for mu, val in w_rules) / w_den if w_den > 0.0 else 0.0
        return v, w

    def control_loop(self):
        if self.x is None or self.goal_reached:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist_err = math.hypot(dx, dy)

        if dist_err <= 0.10:
            self.pub_cmd(0.0, 0.0)
            self.goal_reached = True
            self.get_logger().info(
                f'[HÍBRIDO] Destino alcançado! Posição final: ({self.x:.2f}, {self.y:.2f})'
            )
            return

        # Alternância dinâmica entre PID e Fuzzy conforme leitura do LiDAR
        if self.mode == 'PID' and self.front_dist < self.enter_fuzzy_dist:
            self.mode = 'FUZZY'
            self.fuzzy_hold_ticks = 12  # Contorna por pelo menos 1.2s para sair da linha do pilar
            self.avoid_dir = 1.0 if self.left_dist >= self.right_dist else -1.0
            lado = 'ESQUERDA' if self.avoid_dir > 0 else 'DIREITA'
            self.get_logger().warn(
                f'>>> Obstáculo a {self.front_dist:.2f} m! Alternando para FUZZY (desvio à {lado})'
            )
        elif self.mode == 'FUZZY':
            if self.fuzzy_hold_ticks > 0:
                self.fuzzy_hold_ticks -= 1
            elif self.front_dist > self.exit_fuzzy_dist:
                self.mode = 'PID'
                self.int_dist = 0.0
                self.int_ang = 0.0
                self.get_logger().info(
                    f'<<< Caminho livre ({self.front_dist:.2f} m)! Retornando ao PID de posição'
                )

        if self.mode == 'FUZZY':
            v, w = self.compute_fuzzy_avoidance()
        else:
            desired_yaw = math.atan2(dy, dx)
            ang_err = math.atan2(
                math.sin(desired_yaw - self.yaw),
                math.cos(desired_yaw - self.yaw)
            )
            v, w = self.compute_pid_position(dist_err, ang_err)

        self.pub_cmd(v, w)

        self.tick += 1
        if self.tick % 10 == 0:
            self.get_logger().info(
                f'[{self.mode}] Pos:({self.x:.2f},{self.y:.2f}) | Meta:{dist_err:.2f}m | LiDAR:{self.front_dist:.2f}m'
            )


def main(args=None):
    rclpy.init(args=args)
    node = HybridPidFuzzyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()