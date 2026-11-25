#Assina o tópico /turtle1/pose e exibe x, y e \theta no terminal:
#Implementação de nós ROS 2 para comunicação por tópicos. e Desenvolvimento de um nó responsável pela leitura da posição e orientação da tartaruga.



import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriber(Node):
    def __init__(self): 
        super().__init__('pose_subscriber') #
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
    
    def pose_callback(self,msg:Pose):
        self.get_logger().info(
            f'Posição -> x: {msg.x:.2f} | y: {msg.y:.2f} | theta: {msg.theta:.2f} rad'
        )
        
def main(args=None):
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
    