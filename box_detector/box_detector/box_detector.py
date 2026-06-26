#!/usr/bin/env python3
"""
box_detector.py
---------------
PARTE A del reto: "El Censo".

Nodo ROS2 que detecta cajas con un LiDAR 2D y lleva el censo de cuantas
cajas UNICAS ha visto el robot.

Pipeline:
    /scan  ->  filtrar (inf/nan/rango)  ->  clustering 1D por discontinuidad
           ->  filtrar por ancho ~ caja  ->  centroide (marco robot)
           ->  componer con /odom (marco odom)  ->  deduplicar
           ->  publicar /cajas_avistadas (PoseArray) + MarkerArray (RViz)

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

from box_detector import lidar_utils as lu


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')

        # ---- Parametros (se pueden sobreescribir con params.yaml) ----
        self.declare_parameter('umbral_salto', 0.15)      # m, salto que separa clusters
        self.declare_parameter('min_puntos', 3)           # puntos minimos por cluster
        self.declare_parameter('ancho_caja', 0.20)        # m, ancho nominal de la caja
        self.declare_parameter('tolerancia_ancho', 0.10)  # m, +/- sobre ancho_caja
        self.declare_parameter('rango_max_deteccion', 3.0)  # m, ignora cajas lejanas
        self.declare_parameter('dist_duplicado', 0.30)    # m, mismas cajas si < esto

        self.umbral_salto = self.get_parameter('umbral_salto').value
        self.min_puntos = int(self.get_parameter('min_puntos').value)
        self.ancho_caja = self.get_parameter('ancho_caja').value
        self.tol_ancho = self.get_parameter('tolerancia_ancho').value
        self.rango_max = self.get_parameter('rango_max_deteccion').value
        self.dist_dup = self.get_parameter('dist_duplicado').value

        # ---- Estado ----
        # Pose actual del robot en el marco odom (x, y, yaw).
        self.pose = (0.0, 0.0, 0.0)
        self.tengo_odom = False
        # Censo: lista de centroides unicos en marco odom.
        self.cajas_censo = []

        # ---- Suscriptores y publicadores ----
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.pub_cajas = self.create_publisher(PoseArray, '/cajas_avistadas', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cajas_markers', 10)

        self.get_logger().info('box_detector iniciado. Esperando /scan y /odom...')

    # ------------------------------------------------------------------
    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = lu.yaw_desde_quaternion(q.x, q.y, q.z, q.w)
        self.pose = (p.x, p.y, yaw)
        self.tengo_odom = True

    # ------------------------------------------------------------------
    def cb_scan(self, msg: LaserScan):
        if not self.tengo_odom:
            return  # sin odom no podemos fijar la caja en el mundo

        # 1) Filtrar lecturas invalidas y fuera de rango.
        puntos = lu.filtrar_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max)
        if not puntos:
            return

        # 2) Clustering 1D por discontinuidad de rango.
        clusters = lu.clustering_1d(puntos, self.umbral_salto)

        # 3) Filtrar candidatos por numero de puntos y ancho aparente.
        pose_x, pose_y, pose_yaw = self.pose
        detecciones_odom = []  # centroides en marco odom de este barrido

        for c in clusters:
            if len(c) < self.min_puntos:
                continue
            ancho = lu.ancho_cluster(c)
            if abs(ancho - self.ancho_caja) > self.tol_ancho:
                continue

            cx, cy = lu.centroide_cluster(c)            # marco robot
            if (cx * cx + cy * cy) ** 0.5 > self.rango_max:
                continue

            ox, oy = lu.componer_odom(cx, cy, pose_x, pose_y, pose_yaw)  # marco odom
            detecciones_odom.append((ox, oy))

        # 4) Actualizar el censo deduplicando contra lo ya visto.
        nuevas = 0
        for d in detecciones_odom:
            if not self._ya_censada(d):
                self.cajas_censo.append(d)
                nuevas += 1
        if nuevas:
            self.get_logger().info(
                f'Nuevas cajas: {nuevas}  |  Censo total: {len(self.cajas_censo)}')

        # 5) Publicar resultados.
        self._publicar(msg.header)

    # ------------------------------------------------------------------
    def _ya_censada(self, punto):
        for c in self.cajas_censo:
            if lu.distancia(punto, c) < self.dist_dup:
                return True
        return False

    # ------------------------------------------------------------------
    def _publicar(self, header):
        # PoseArray con el censo completo, en marco odom.
        pa = PoseArray()
        pa.header.stamp = header.stamp
        pa.header.frame_id = 'odom'
        for (x, y) in self.cajas_censo:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_cajas.publish(pa)

        # MarkerArray para visualizar en RViz (cubos amarillos + texto con id).
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.cajas_censo):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = header.stamp
            m.ns = 'cajas'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = self.ancho_caja
            m.scale.y = self.ancho_caja
            m.scale.z = 0.2
            m.color.r = 0.96
            m.color.g = 0.62
            m.color.b = 0.04
            m.color.a = 0.9
            ma.markers.append(m)
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
            f'Censo final: {len(nodo.cajas_censo)} cajas unicas.')
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
