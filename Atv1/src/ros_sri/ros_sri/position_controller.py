import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class PositionController(Node):
    def __init__(self, goal_x: float, goal_y: float):
        super().__init__('position_controller')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        self.timers = self.create_timer(0.1, self.timer_callback)   
        self.get_logger().info(
            f'Controlador iniciado. Indo para ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )
    def pose_callback(self, msg: Pose):
        self.current_pose = msg
        
    def control_loop(self):
        if self.current_pose is None or self.goal_reached:
            return

        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        distance_error = math.hypot(dx, dy)

        cmd = Twist()
        
        if distance_error <= 0.05:
            self.cmd_pub.publish(cmd)
            self.goal_reached = True
            self.get_logger().info(
                f'Destino alcançado! Posição final: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})'
            )
            return
        
        desired_theta = math.atan2(dy, dx)
        angle_error = math.atan2(
            math.sin(desired_theta - self.current_pose.theta),
            math.cos(desired_theta - self.current_pose.theta),
        )

        cmd.angular.z = 4.0 * angle_error
        cmd.linear.x = min(1.5 * distance_error, 2.0) if abs(angle_error) < 0.35 else 0.0
        self.cmd_pub.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    try:
        goal_x = float(input('Digite a coordenada X desejada (entre 0.5 e 10.5): '))
        goal_y = float(input('Digite a coordenada Y desejada (entre 0.5 e 10.5): '))
    except ValueError:
        print('Entrada inválida. Usando destino padrão (8.0, 8.0).')
        goal_x, goal_y = 8.0, 8.0

    node = PositionController(goal_x, goal_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()