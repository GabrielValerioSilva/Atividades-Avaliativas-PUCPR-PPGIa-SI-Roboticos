import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
import matplotlib.pyplot as plt


# Aplica a transformação rígida 3D: rotação pela quaternion e translação da TF.
def transform_points(pts: np.ndarray, tf_stamped) -> np.ndarray:
    t = tf_stamped.transform
    qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    rot = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)]
    ])
    trans = np.array([t.translation.x, t.translation.y, t.translation.z])
    return pts @ rot.T + trans


class Tf2LidarMapper(Node):
    def __init__(self):
        super().__init__('tf2_lidar_mapper')
        # Buffer armazena TFs recebidas; listener mantém a árvore de transformações atualizada.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data
        )

        # Resolução de 3 cm agrupa pontos próximos em células para reduzir duplicatas no mapa.
        self.res = 0.03
        self.voxels = set()
        self.global_pts = []
        self.live_pts = np.empty((0, 2))
        self.traj_x, self.traj_y = [], []
        self.pose = (0.0, 0.0)
        self.count = 0

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.g_scat = self.ax.scatter([], [], s=5, c='royalblue', alpha=0.7, label='Mapa Global (odom)')
        self.l_scat = self.ax.scatter([], [], s=10, c='limegreen', label='Scan Atual')
        self.t_line, = self.ax.plot([], [], 'r--', linewidth=1.2, label='Trajetória')
        self.r_mark = self.ax.scatter([0.0], [0.0], s=85, c='red', marker='^', zorder=5, label='Robô')

        self.ax.set_title('Mapa Global via TF2 (Referencial: odom)')
        self.ax.set_xlabel('X [m] (odom)')
        self.ax.set_ylabel('Y [m] (odom)')
        self.ax.set_xlim(-3.5, 3.5)
        self.ax.set_ylim(-3.5, 3.5)
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.legend(loc='upper right')

        self.timer = self.create_timer(0.3, self.update_plot)
        self.get_logger().info('Mapeador Global TF2 iniciado!')

    def scan_cb(self, msg: LaserScan):
        # Converte cada leitura polar válida do LiDAR para coordenadas cartesianas do sensor.
        src = msg.header.frame_id
        pts = []
        for i, d in enumerate(msg.ranges):
            if math.isinf(d) or math.isnan(d) or d < msg.range_min or d > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            pts.append([d * math.cos(a), d * math.sin(a), 0.0])

        if not pts:
            return
        pts_sensor = np.array(pts)

        # Compara o scan nos frames base_link e odom e obtém a pose do robô em odom.
        try:
            tf_base = self.tf_buffer.lookup_transform('base_link', src, rclpy.time.Time())
            tf_odom = self.tf_buffer.lookup_transform('odom', src, rclpy.time.Time())
            tf_robot = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Aguardando TF2: {ex}', throttle_duration_sec=2.0)
            return

        # TF2 move os pontos do sensor para os frames do robô e do mapa global odom.
        pts_base = transform_points(pts_sensor, tf_base)
        pts_odom = transform_points(pts_sensor, tf_odom)
        self.live_pts = pts_odom[:, :2]

        # Set de índices de voxel evita inserir repetidamente a mesma célula de 3 cm.
        for p in self.live_pts:
            k = (round(float(p[0]) / self.res), round(float(p[1]) / self.res))
            if k not in self.voxels:
                self.voxels.add(k)
                self.global_pts.append([k[0] * self.res, k[1] * self.res])

        rx, ry = tf_robot.transform.translation.x, tf_robot.transform.translation.y
        self.pose = (rx, ry)
        self.traj_x.append(rx)
        self.traj_y.append(ry)

        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(
                f'TF2 ({src}->base_link->odom) | base:({pts_base[0,0]:.2f},{pts_base[0,1]:.2f}) '
                f'-> odom:({pts_odom[0,0]:.2f},{pts_odom[0,1]:.2f}) | Pontos: {len(self.global_pts)}'
            )

    def update_plot(self):
        # Atualiza nuvem global, scan atual e trajetória sem bloquear o executor ROS.
        if not self.global_pts:
            return
        self.g_scat.set_offsets(np.array(self.global_pts))
        self.l_scat.set_offsets(self.live_pts)
        self.t_line.set_data(self.traj_x, self.traj_y)
        self.r_mark.set_offsets([[self.pose[0], self.pose[1]]])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = Tf2LidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()