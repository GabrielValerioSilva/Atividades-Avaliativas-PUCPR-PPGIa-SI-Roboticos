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
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data
        )
        self.turning = False
        self.turn_dir = 1.0
        self.scan = None
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info('Navegação reativa iniciada!')

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def pub_cmd(self, vx: float, wz: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(vx)
        cmd.twist.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    def loop(self):
        if self.scan is None:
            return
        msg = self.scan
        front, left, right = [], [], []
        for i, d in enumerate(msg.ranges):
            if math.isinf(d) or math.isnan(d) or d < msg.range_min or d > msg.range_max:
                continue
            a = math.atan2(
                math.sin(msg.angle_min + i * msg.angle_increment),
                math.cos(msg.angle_min + i * msg.angle_increment)
            )
            if -0.52 <= a <= 0.52:
                front.append(d)
            elif 0.52 < a <= 1.75:
                left.append(d)
            elif -1.75 <= a < -0.52:
                right.append(d)

        min_f = min(front) if front else msg.range_max
        avg_l = (sum(left) / len(left)) if left else 0.0
        avg_r = (sum(right) / len(right)) if right else 0.0

        if not self.turning and min_f < 0.55:
            self.turning = True
            self.turn_dir = 1.0 if avg_l >= avg_r else -1.0
        elif self.turning and min_f > 0.75:
            self.turning = False

        if self.turning:
            self.pub_cmd(0.0, self.turn_dir * 0.50)
        else:
            self.pub_cmd(0.12, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigation()
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