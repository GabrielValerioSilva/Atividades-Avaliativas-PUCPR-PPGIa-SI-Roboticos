import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquarePublisher(Node):
    def __init__(self):
        super().__init__('square_publisher')
        # Comandos Twist controlam velocidade linear e angular da tartaruga.
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Um período de 0,1 s corresponde a 10 atualizações de comando por segundo.
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Máquina de estados alterna avanço e giro até completar quatro lados.
        self.state = 'FORWARD'
        self.elapsed = 0.0
        self.sides_completed = 0
        self.get_logger().info('Iniciando desenho do quadrado...')

    def timer_callback(self):
        # Um Twist zerado interrompe o movimento ao encerrar o quadrado.
        msg = Twist()

        if self.sides_completed >= 4:
            self.publisher_.publish(msg)
            self.get_logger().info('Quadrado concluído!')
            self.timer.cancel()
            return

        self.elapsed += self.timer_period

        # O avanço dura 2 s; depois, o estado TURN aplica pi/2 rad/s por 1 s (aprox. 90 graus).
        if self.state == 'FORWARD':
            if self.elapsed <= 2.0:
                msg.linear.x = 1.5
            else:
                self.state = 'TURN'
                self.elapsed = 0.0

        elif self.state == 'TURN':
            if self.elapsed <= 1.0:
                msg.angular.z = math.pi / 2.0
            else:
                self.sides_completed += 1
                self.state = 'FORWARD'
                self.elapsed = 0.0

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquarePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()