import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class ReactiveNavigation(Node):
    def __init__(self):
        super().__init__('reactive_navigation')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )

        self.linear_speed = 0.10       # Velocidade suave e constante (m/s)
        self.angular_speed = 0.45      # Giro suave (rad/s)
        self.stop_distance = 0.55      # Começa a desviar abaixo de 0.55 m
        self.clear_distance = 0.75     # Volta a andar reto acima de 0.75 m
        self.turning = False
        self.turn_dir = 1.0
        self.latest_scan = None
        self.tick = 0

        self.timer = self.create_timer(0.1, self.control_loop)  # Loop fixo a 10 Hz
        self.get_logger().info('Navegação reativa suave iniciada!')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def publish_cmd(self, linear_x: float, angular_z: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if self.latest_scan is None:
            return

        msg = self.latest_scan
        front, left, right = [], [], []

        for i, d in enumerate(msg.ranges):
            if math.isinf(d) or math.isnan(d) or d < msg.range_min or d > msg.range_max:
                continue
            raw_angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))

            if -0.52 <= angle <= 0.52:
                front.append(d)
            elif 0.52 < angle <= 1.75:
                left.append(d)
            elif -1.75 <= angle < -0.52:
                right.append(d)

        min_front = min(front) if front else msg.range_max
        avg_left = (sum(left) / len(left)) if left else 0.0
        avg_right = (sum(right) / len(right)) if right else 0.0

        if not self.turning and min_front < self.stop_distance:
            self.turning = True
            self.turn_dir = 1.0 if avg_left >= avg_right else -1.0
        elif self.turning and min_front > self.clear_distance:
            self.turning = False

        if self.turning:
            self.publish_cmd(0.0, self.turn_dir * self.angular_speed)
            status = 'DESVIANDO (ESQ)' if self.turn_dir > 0 else 'DESVIANDO (DIR)'
        else:
            self.publish_cmd(self.linear_speed, 0.0)
            status = 'ANDANDO RETO'

        self.tick += 1
        if self.tick % 10 == 0:
            self.get_logger().info(f'Frente: {min_front:.2f} m | {status}')


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
