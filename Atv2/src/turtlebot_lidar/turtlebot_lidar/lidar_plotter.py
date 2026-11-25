import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


class LidarPlotter(Node):
    def __init__(self):
        super().__init__('lidar_plotter')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.points = np.empty((0, 2))

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(5, 5))
        self.scatter = self.ax.scatter([], [], s=8, c='blue', label='Obstáculos (LiDAR)')
        self.ax.scatter([0.0], [0.0], s=80, c='red', marker='^', label='TurtleBot3')
        self.ax.set_title('Mapa Local 2D - Coordenadas Cartesianas (LiDAR)')
        self.ax.set_xlabel('X [m] (Frente ->)')
        self.ax.set_ylabel('Y [m] (Esquerda ^)')
        self.ax.set_xlim(-4.0, 4.0)
        self.ax.set_ylim(-4.0, 4.0)
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.legend(loc='upper right')

        self.timer = self.create_timer(0.3, self.update_plot)
        self.get_logger().info('Plotador 2D do LiDAR iniciado!')

    def scan_callback(self, msg: LaserScan):
        xs, ys = [], []
        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            xs.append(distance * math.cos(angle))
            ys.append(distance * math.sin(angle))
        if xs:
            self.points = np.column_stack((xs, ys))

    def update_plot(self):
        if len(self.points) == 0:
            return
        self.scatter.set_offsets(self.points)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = LidarPlotter()
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
