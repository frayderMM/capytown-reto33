"""
box_detector.py — Parte A del RC3
Detecta cajas de cartón en el LiDAR usando clustering 1D por salto de rango.
Publica centroides en /cajas_avistadas y markers para RViz en /cajas_markers.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')

        # --- parámetros tuneables ---
        self.declare_parameter('jump_threshold', 0.12)   # salto de rango [m] para abrir cluster
        self.declare_parameter('box_min_width', 0.10)    # ancho mínimo de caja [m]
        self.declare_parameter('box_max_width', 0.35)    # ancho máximo de caja [m]
        self.declare_parameter('box_min_points', 3)      # puntos mínimos por cluster
        self.declare_parameter('box_max_points', 60)     # puntos máximos por cluster
        self.declare_parameter('max_range', 3.0)         # distancia máxima de detección [m]

        self.jump_thr    = self.get_parameter('jump_threshold').value
        self.box_min_w   = self.get_parameter('box_min_width').value
        self.box_max_w   = self.get_parameter('box_max_width').value
        self.box_min_pts = self.get_parameter('box_min_points').value
        self.box_max_pts = self.get_parameter('box_max_points').value
        self.max_range   = self.get_parameter('max_range').value

        # pose del robot leída de /odom
        self.rx, self.ry, self.ryaw = 0.0, 0.0, 0.0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)

        self.poses_pub   = self.create_publisher(PoseArray,   '/cajas_avistadas', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/cajas_markers',   10)

        self.get_logger().info('BoxDetector listo')

    # ------------------------------------------------------------------ odom
    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        self.rx  = pos.x
        self.ry  = pos.y
        # yaw desde cuaternión
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.ryaw = math.atan2(siny, cosy)

    # ------------------------------------------------------------------ scan
    def _scan_cb(self, msg):
        a0  = msg.angle_min
        da  = msg.angle_increment
        rmin = msg.range_min
        rmax = min(msg.range_max, self.max_range)

        # --- filtrar puntos válidos ---
        valid = []
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and rmin < r < rmax:
                valid.append((r, a0 + i * da))

        if len(valid) < 2:
            return

        # --- clustering 1D por salto de rango ---
        clusters, cur = [], [valid[0]]
        for i in range(1, len(valid)):
            if abs(valid[i][0] - valid[i-1][0]) > self.jump_thr:
                clusters.append(cur)
                cur = []
            cur.append(valid[i])
        clusters.append(cur)

        # --- validar cada cluster ---
        boxes = []
        for clust in clusters:
            n = len(clust)
            if not (self.box_min_pts <= n <= self.box_max_pts):
                continue

            # convertir a cartesiano (frame base_link)
            xs = [r * math.cos(th) for r, th in clust]
            ys = [r * math.sin(th) for r, th in clust]

            # ancho aparente = distancia entre primer y último punto
            width = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
            if not (self.box_min_w <= width <= self.box_max_w):
                continue

            # centroide en base_link
            cx_bl = sum(xs) / n
            cy_bl = sum(ys) / n

            # transformar a frame odom
            c = math.cos(self.ryaw)
            s = math.sin(self.ryaw)
            cx_odom = self.rx + c * cx_bl - s * cy_bl
            cy_odom = self.ry + s * cx_bl + c * cy_bl

            boxes.append((cx_odom, cy_odom))
            self.get_logger().debug(
                f'Caja detectada: ({cx_odom:.2f}, {cy_odom:.2f}) m | ancho={width:.2f} m | pts={n}')

        self._publish(boxes, msg.header)

    # ------------------------------------------------------------------ publish
    def _publish(self, boxes, header):
        pa = PoseArray()
        pa.header.frame_id = 'odom'
        pa.header.stamp    = header.stamp

        ma = MarkerArray()

        for i, (x, y) in enumerate(boxes):
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.orientation.w = 1.0
            pa.poses.append(p)

            mk = Marker()
            mk.header.frame_id = 'odom'
            mk.header.stamp    = header.stamp
            mk.ns      = 'cajas'
            mk.id      = i
            mk.type    = Marker.CUBE
            mk.action  = Marker.ADD
            mk.pose.position.x  = x
            mk.pose.position.y  = y
            mk.pose.position.z  = 0.10
            mk.pose.orientation.w = 1.0
            mk.scale.x = 0.20
            mk.scale.y = 0.20
            mk.scale.z = 0.20
            mk.color.r = 1.0
            mk.color.g = 0.5
            mk.color.b = 0.0
            mk.color.a = 0.85
            mk.lifetime.sec = 1   # desaparece si no se refresca
            ma.markers.append(mk)

        self.poses_pub.publish(pa)
        self.markers_pub.publish(ma)

        if boxes:
            self.get_logger().info(f'{len(boxes)} caja(s) publicadas en /cajas_avistadas')


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
