import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# Publica quatro PoseStamped em /goal_pose e monitora /odom até cada meta ser alcançada.
class Nav2FourGoals(Node):
    def __init__(self) -> None:
        super().__init__('nav2_four_goals')

        # Cada meta é configurada em metros e yaw em radianos, no frame global map.
        for index in range(1, 5):
            self.declare_parameter(f'goal_{index}_x', 0.0)
            self.declare_parameter(f'goal_{index}_y', 0.0)
            self.declare_parameter(f'goal_{index}_yaw', 0.0)

        # Tolerância espacial e limite de tempo evitam avanço prematuro ou espera infinita.
        self.declare_parameter('position_tolerance', 0.25)
        self.declare_parameter('goal_timeout_sec', 180.0)

        self.goals: List[Dict[str, float]] = []
        for index in range(1, 5):
            self.goals.append({
                'x': float(self.get_parameter(f'goal_{index}_x').value),
                'y': float(self.get_parameter(f'goal_{index}_y').value),
                'yaw': float(self.get_parameter(f'goal_{index}_yaw').value),
            })

        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )
        self.goal_timeout_sec = float(
            self.get_parameter('goal_timeout_sec').value
        )

        # Nav2 recebe metas PoseStamped pelo tópico usado também pela ferramenta de RViz.
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10,
        )
        # /odom é a fonte de pose pedida no tutorial; TF2 a converte para o frame map.
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # O buffer acompanha map->odom, publicado após a localização inicial no RViz.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_map_position: Optional[Tuple[float, float]] = None
        self.goal_index = 0
        self.goal_active = False
        self.goal_started_at = None
        self.within_tolerance_count = 0
        self.tour_complete = False
        self.tour_failed = False

        self.create_timer(0.2, self.advance_tour)

    @staticmethod
    def quaternion_yaw(quaternion) -> float:
        # Extrai a rotação em torno de Z de uma quaternion para transformar coordenadas 2D.
        return math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    def odom_callback(self, msg: Odometry) -> None:
        # Ignora odometria sem frame declarado: não é possível relacioná-la corretamente ao mapa.
        if not msg.header.frame_id:
            self.get_logger().warning(
                'Mensagem /odom sem frame_id; pose ignorada.',
                throttle_duration_sec=5.0,
            )
            return

        # Solicita a transformação mais recente entre o frame da odometria e map.
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Aguardando transformacao de map para '
                f'{msg.header.frame_id}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        transform_yaw = self.quaternion_yaw(transform.transform.rotation)
        cosine = math.cos(transform_yaw)
        sine = math.sin(transform_yaw)
        odom_pose = msg.pose.pose.position
        translation = transform.transform.translation

        # Aplica rotação planar e translação da TF para comparar pose e meta no mesmo frame.
        map_x = translation.x + cosine * odom_pose.x - sine * odom_pose.y
        map_y = translation.y + sine * odom_pose.x + cosine * odom_pose.y
        self.current_map_position = (map_x, map_y)

        if not self.goal_active or self.goal_index >= len(self.goals):
            return

        goal = self.goals[self.goal_index]
        distance = math.hypot(map_x - goal['x'], map_y - goal['y'])
        self.get_logger().info(
            f'Meta {self.goal_index + 1}: distancia atual '
            f'{distance:.2f} m.',
            throttle_duration_sec=2.0,
        )

        # Exige três amostras consecutivas dentro da tolerância para reduzir falsos alcances.
        if distance <= self.position_tolerance:
            self.within_tolerance_count += 1
            if self.within_tolerance_count >= 3:
                self.complete_current_goal(distance)
        else:
            self.within_tolerance_count = 0

    def advance_tour(self) -> None:
        # O timer só envia a próxima meta após a confirmação da atual ou interrompe no timeout.
        if self.tour_complete or self.tour_failed or self.goal_active:
            if self.goal_active and self.goal_started_at is not None:
                elapsed = (
                    self.get_clock().now() - self.goal_started_at
                ).nanoseconds / 1e9
                if elapsed > self.goal_timeout_sec:
                    self.tour_failed = True
                    self.get_logger().error(
                        f'Tempo limite excedido na meta {self.goal_index + 1}; '
                        'percurso interrompido.'
                    )
            return

        if self.goal_index >= len(self.goals):
            self.tour_complete = True
            self.get_logger().info(
                'Percurso concluido: as quatro metas foram alcancadas.'
            )
            return

        if self.current_map_position is None:
            self.get_logger().info(
                'Aguardando /odom e a localizacao do Nav2 no frame map...',
                throttle_duration_sec=5.0,
            )
            return

        # Aguarda o Nav2 assinar /goal_pose para não perder a primeira publicação.
        if self.goal_publisher.get_subscription_count() == 0:
            self.get_logger().info(
                'Aguardando o Nav2 assinar o topico /goal_pose...',
                throttle_duration_sec=5.0,
            )
            return

        self.publish_current_goal()

    def publish_current_goal(self) -> None:
        # A meta usa frame map; yaw é codificado na quaternion planar z/w.
        goal = self.goals[self.goal_index]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal['x']
        msg.pose.position.y = goal['y']
        msg.pose.orientation.z = math.sin(goal['yaw'] / 2.0)
        msg.pose.orientation.w = math.cos(goal['yaw'] / 2.0)

        self.goal_publisher.publish(msg)
        self.goal_active = True
        self.goal_started_at = self.get_clock().now()
        self.within_tolerance_count = 0
        self.get_logger().info(
            f"Meta {self.goal_index + 1}/4 publicada em /goal_pose: "
            f"x={goal['x']:.2f}, y={goal['y']:.2f}, "
            f"yaw={goal['yaw']:.2f} rad."
        )

    def complete_current_goal(self, distance: float) -> None:
        # Avança o índice apenas após atingir a tolerância e registra o resultado no log.
        self.get_logger().info(
            f'Meta {self.goal_index + 1}/4 alcancada '
            f'(distancia={distance:.2f} m).'
        )
        self.goal_active = False
        self.goal_started_at = None
        self.within_tolerance_count = 0
        self.goal_index += 1
        if self.goal_index >= len(self.goals):
            self.tour_complete = True
            self.get_logger().info(
                'Percurso concluido: as quatro metas foram alcancadas.'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2FourGoals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
