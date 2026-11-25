import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
#Controlador PID
#para levar o TurtleBot3 até (goal_x, goal_y) usando a odometria /odom


class PidPositionController(Node):
    def __init__(self):
        super().__init__('pid_position_controller')

        # Coordenadas desejadas configuráveis via parâmetro ROS 2
        self.declare_parameter('goal_x', 0.5)
        self.declare_parameter('goal_y', 0.5)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.x = None
        self.y = None
        self.yaw = 0.0
        self.goal_reached = False

        # Ganhos PID (Linear e Angular)
        self.kp_lin, self.ki_lin, self.kd_lin = 0.45, 0.02, 0.10
        self.kp_ang, self.ki_ang, self.kd_ang = 1.40, 0.05, 0.20

        self.int_dist = 0.0
        self.prev_dist = 0.0
        self.int_ang = 0.0
        self.prev_ang = 0.0
        self.dt = 0.1
        self.tick = 0

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'Controlador PID iniciado! Destino: ({self.goal_x:.2f}, {self.goal_y:.2f})'
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
                f'[PID] Destino alcançado! Posição final: ({self.x:.2f}, {self.y:.2f})'
            )
            return

        desired_yaw = math.atan2(dy, dx)
        ang_err = math.atan2(
            math.sin(desired_yaw - self.yaw),
            math.cos(desired_yaw - self.yaw)
        )

        # Termos Integral (com anti-windup) e Derivativo
        self.int_dist = max(-0.5, min(0.5, self.int_dist + dist_err * self.dt))
        der_dist = (dist_err - self.prev_dist) / self.dt
        self.prev_dist = dist_err

        self.int_ang = max(-0.5, min(0.5, self.int_ang + ang_err * self.dt))
        der_ang = (ang_err - self.prev_ang) / self.dt
        self.prev_ang = ang_err

        # Saída PID saturada nos limites seguros do TurtleBot3
        w = self.kp_ang * ang_err + self.ki_ang * self.int_ang + self.kd_ang * der_ang
        w = max(-0.75, min(0.75, w))

        if abs(ang_err) < 0.45:
            v = self.kp_lin * dist_err + self.ki_lin * self.int_dist + self.kd_lin * der_dist
            v = max(0.0, min(0.15, v))
        else:
            v = 0.0  # Alinha orientação antes de acelerar

        self.pub_cmd(v, w)

        self.tick += 1
        if self.tick % 10 == 0:
            self.get_logger().info(
                f'[PID] Pos: ({self.x:.2f}, {self.y:.2f}) | Erro dist: {dist_err:.2f} m | v={v:.2f}, w={w:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PidPositionController()
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