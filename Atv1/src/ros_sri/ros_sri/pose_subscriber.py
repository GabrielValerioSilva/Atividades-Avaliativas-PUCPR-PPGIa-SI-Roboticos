#Assina o tópico /turtle1/pose e exibe x, y e \theta no terminal:
#Implementação de nós ROS 2 para comunicação por tópicos. e Desenvolvimento de um nó responsável pela leitura da posição e orientação da tartaruga.

# Este nó demonstra o padrão publisher/subscriber: recebe mensagens Pose do turtlesim.

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriber(Node):
    def __init__(self): 
        # O nome identifica o nó na rede ROS 2; a assinatura recebe até 10 mensagens em fila.
        super().__init__('pose_subscriber') #
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
    
    def pose_callback(self,msg:Pose):
        # A Pose já contém posição cartesiana e orientação em radianos.
        self.get_logger().info(
            f'Posição -> x: {msg.x:.2f} | y: {msg.y:.2f} | theta: {msg.theta:.2f} rad'
        )
        
def main(args=None):
    # Inicializa ROS 2, mantém o assinante processando callbacks e libera os recursos ao sair.
    rclpy.init(args=args)
    node = PoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    