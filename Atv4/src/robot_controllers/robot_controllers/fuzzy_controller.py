import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
#Controlador baseado em lógica Fuzzy
#conduzir o robô até a mesma posição desejada


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


class FuzzyPositionController(Node):
    def __init__(self):
        super().__init__('fuzzy_position_controller')

        self.declare_parameter('goal_x', -0.5)
        self.declare_parameter('goal_y', -0.5)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.x = None
        self.y = None
        self.yaw = 0.0
        self.goal_reached = False
        self.tick = 0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Controlador FUZZY iniciado! Destino: ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def pub_cmd(self, vx: float, wz: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(vx)
        cmd.twist.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    def evaluate_fuzzy(self, dist_err: float, ang_err: float):
        # 1. Fuzzificação do erro de distância (m)
        d_perto = trapmf(dist_err, -0.1, 0.0, 0.12, 0.35)
        d_medio = trimf(dist_err, 0.15, 0.55, 1.10)
        d_longe = trapmf(dist_err, 0.70, 1.30, 10.0, 11.0)

        # 2. Fuzzificação do erro angular (rad)
        a_dir_f = trapmf(ang_err, -4.0, -3.5, -0.90, -0.45)
        a_dir = trimf(ang_err, -0.75, -0.35, -0.05)
        a_centro = trimf(ang_err, -0.25, 0.0, 0.25)
        a_esq = trimf(ang_err, 0.05, 0.35, 0.75)
        a_esq_f = trapmf(ang_err, 0.45, 0.90, 3.5, 4.0)

        # 3. Base de Regras Fuzzy (Inferência Mamdani - operador min)
        # Regras para velocidade angular (singletons de saída em rad/s)
        w_rules = [
            (a_dir_f, -0.75),
            (a_dir, -0.38),
            (a_centro, 0.0),
            (a_esq, 0.38),
            (a_esq_f, 0.75),
        ]
        w_num = sum(mu * val for mu, val in w_rules)
        w_den = sum(mu for mu, _ in w_rules)
        w_out = (w_num / w_den) if w_den > 0.0 else 0.0

        # Regras para velocidade linear (singletons de saída em m/s)
        v_rules = [
            (min(d_longe, a_centro), 0.15),
            (min(d_medio, a_centro), 0.10),
            (min(d_perto, a_centro), 0.05),
            (min(d_longe, max(a_esq, a_dir)), 0.08),
            (min(d_medio, max(a_esq, a_dir)), 0.05),
            (min(d_perto, max(a_esq, a_dir)), 0.03),
            (max(a_esq_f, a_dir_f), 0.0),
        ]
        v_num = sum(mu * val for mu, val in v_rules)
        v_den = sum(mu for mu, _ in v_rules)
        v_out = (v_num / v_den) if v_den > 0.0 else 0.0

        return v_out, w_out

    def control_loop(self):
        if self.x is None or self.goal_reached:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist_err = math.hypot(dx, dy)

        if dist_err <= 0.06:
            self.pub_cmd(0.0, 0.0)
            self.goal_reached = True
            self.get_logger().info(
                f'[FUZZY] Destino alcançado! Posição final: ({self.x:.2f}, {self.y:.2f})'
            )
            return

        desired_yaw = math.atan2(dy, dx)
        ang_err = math.atan2(
            math.sin(desired_yaw - self.yaw),
            math.cos(desired_yaw - self.yaw)
        )

        v, w = self.evaluate_fuzzy(dist_err, ang_err)
        self.pub_cmd(v, w)

        self.tick += 1
        if self.tick % 10 == 0:
            self.get_logger().info(
                f'[FUZZY] Pos: ({self.x:.2f}, {self.y:.2f}) | Erro dist: {dist_err:.2f} m | v={v:.2f}, w={w:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyPositionController()
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