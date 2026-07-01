#!/usr/bin/env python3
"""
box_detector.py — PARTE A: "El Censo" (versión corregida).

Pipeline:
    /scan → rotar al marco del robot (lidar_front_deg=180, MS200 Yahboom)
          → filtrar (inf/nan/rango/cono trasero del cable)
          → clustering por salto euclidiano
          → Split-and-Merge (bug del [1:] corregido) → lados del cluster
          → caja ⇔ TODOS los lados en [lado_caja_min, lado_caja_max]
          → centroide (base_link) → componer con /odom_raw (marco odom)
          → deduplicar → /cajas_avistadas (PoseArray) + /cajas_markers (RViz)

CORRECCIONES respecto a la versión anterior:
  · El scan se rota al marco del robot ANTES de todo — antes el censo se
    componía con /odom en un marco girado 180° y las posiciones salían
    espejadas (error de posición sistemático enorme).
  · QoS BEST_EFFORT en /scan (el driver del MS200 publica best-effort;
    con QoS por defecto RELIABLE el nodo podía no recibir NADA).
  · Validación por LADOS del cluster, no por ancho extremo-a-extremo:
    rechaza paredes fragmentadas (FP) sin rechazar cajas vistas en L.
  · Solo se censan clusters vistos con odometría disponible.

ESAN - Robótica de Móviles 2026-I | Proyecto CapyTown
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

from box_detector import lidar_utils as lu


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')

        # ── Parámetros ──────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg', 180.0)
        self.declare_parameter('excluir_atras_deg', 60.0)
        self.declare_parameter('umbral_salto', 0.12)     # m salto euclidiano
        self.declare_parameter('salto_idx', 5)
        self.declare_parameter('umbral_split', 0.04)     # m rectitud S&M
        self.declare_parameter('min_puntos', 4)
        self.declare_parameter('lado_caja_min', 0.08)    # m lado mínimo de caja
        self.declare_parameter('lado_caja_max', 0.32)    # m lado máx (20 + tol)
        self.declare_parameter('rango_max_deteccion', 2.5)
        self.declare_parameter('dist_duplicado', 0.30)
        self.declare_parameter('confirmaciones_caja', 3)
        self.declare_parameter('max_cajas', 8)
        self.declare_parameter('topic_odom', '/odom_raw')

        g = lambda n: self.get_parameter(n).value
        self.front_rad = math.radians(g('lidar_front_deg'))
        self.atras_rad = math.radians(g('excluir_atras_deg')) / 2.0
        self.salto = g('umbral_salto')
        self.salto_idx = int(g('salto_idx'))
        self.umbral_split = g('umbral_split')
        self.min_puntos = int(g('min_puntos'))
        self.lado_min = g('lado_caja_min')
        self.lado_max = g('lado_caja_max')
        self.rango_max = g('rango_max_deteccion')
        self.dist_dup = g('dist_duplicado')
        self.confirmaciones = int(g('confirmaciones_caja'))
        self.max_cajas = int(g('max_cajas'))

        # ── Estado ──────────────────────────────────────────────────────
        self.pose = None            # (x, y, yaw) en odom
        self.censo = []             # centroides únicos en marco odom
        self.pendientes = []

        # ── ROS I/O ─────────────────────────────────────────────────────
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos)
        self.create_subscription(Odometry, g('topic_odom'), self.cb_odom, qos)
        self.pub_cajas = self.create_publisher(PoseArray, '/cajas_avistadas', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cajas_markers', 10)

        self.get_logger().info(
            f'box_detector listo | frente LiDAR {g("lidar_front_deg"):.0f}° '
            f'| caja: lados en [{self.lado_min:.2f}, {self.lado_max:.2f}] m '
            f'| odom: {g("topic_odom")}')

    # ------------------------------------------------------------------
    def cb_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y,
                     lu.yaw_desde_quaternion(q.x, q.y, q.z, q.w))

    # ------------------------------------------------------------------
    def cb_scan(self, msg: LaserScan):
        if self.pose is None:
            self.get_logger().warn('sin /odom aún — no se puede censar',
                                   throttle_duration_sec=5.0)
            return

        pts = lu.filtrar_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max,
                              self.front_rad, self.rango_max, self.atras_rad)
        px, py, pyaw = self.pose

        observadas = []
        for grupo in lu.pre_segmentar(pts, self.salto, self.salto_idx):
            if len(grupo) < self.min_puntos:
                continue
            if not lu.es_caja(grupo, self.umbral_split, self.min_puntos,
                              self.lado_min, self.lado_max):
                continue
            cx, cy = lu.centroide(grupo)
            if math.hypot(cx, cy) > self.rango_max:
                continue
            ox, oy = lu.componer_odom(cx, cy, px, py, pyaw)
            observadas.append((ox, oy))

        nuevas = self._confirmar_observaciones(observadas)

        if nuevas:
            self.get_logger().info(
                f'Nuevas cajas: {nuevas} | Censo total: {len(self.censo)}')
        self._publicar(msg.header)

    # ------------------------------------------------------------------
    def _confirmar_observaciones(self, observadas):
        nuevas = 0
        for p in self.pendientes:
            p['miss'] += 1

        for ox, oy in observadas:
            if any(lu.distancia((ox, oy), c) < self.dist_dup
                   for c in self.censo):
                continue
            mejor = None
            for p in self.pendientes:
                d = lu.distancia((ox, oy), (p['x'], p['y']))
                if d < self.dist_dup and (mejor is None or d < mejor[0]):
                    mejor = (d, p)
            if mejor is None:
                self.pendientes.append({'x': ox, 'y': oy, 'hits': 1, 'miss': 0})
                continue
            p = mejor[1]
            h = p['hits']
            p['x'] = (p['x'] * h + ox) / (h + 1)
            p['y'] = (p['y'] * h + oy) / (h + 1)
            p['hits'] = h + 1
            p['miss'] = 0
            if p['hits'] >= self.confirmaciones and len(self.censo) < self.max_cajas:
                self.censo.append((p['x'], p['y']))
                nuevas += 1

        self.pendientes = [
            p for p in self.pendientes
            if p['miss'] <= 8 and p['hits'] < self.confirmaciones
        ]
        return nuevas

    # ------------------------------------------------------------------
    def _publicar(self, header):
        pa = PoseArray()
        pa.header.stamp = header.stamp
        pa.header.frame_id = 'odom'
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.censo):
            p = Pose()
            p.position.x, p.position.y = x, y
            p.orientation.w = 1.0
            pa.poses.append(p)

            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = header.stamp
            m.ns, m.id = 'cajas', i
            m.type, m.action = Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.20
            m.color.r, m.color.g, m.color.b, m.color.a = 0.96, 0.62, 0.04, 0.9
            ma.markers.append(m)
        self.pub_cajas.publish(pa)
        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    nodo = BoxDetector()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.get_logger().info(
            f'Censo final: {len(nodo.censo)} cajas únicas.')
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
