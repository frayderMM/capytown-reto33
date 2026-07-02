#!/usr/bin/env python3
"""
wall_follower.py — Detección de paredes laterales por RANSAC (RC3).

Suscribe a /scan, ajusta una recta por RANSAC a los puntos de cada lado
(izquierda / derecha) del jirón y publica la distancia perpendicular del
robot a cada pared en /dist_izq y /dist_der (std_msgs/Float32). behavior_fsm
consume estos dos tópicos para el tracking lateral; el frente (parada de
seguridad) lo mide behavior_fsm directamente sobre /scan, sin saltos entre
nodos, para no añadir latencia a esa reacción crítica.

Por qué RANSAC y no Split-and-Merge:
    S&M necesita que los puntos de la pared sean contiguos. Si una caja
    interrumpe el tramo de pared que ve el LiDAR, el segmento se corta o
    se fusiona mal. RANSAC ajusta por consenso: los puntos de la caja
    quedan fuera del modelo (outliers) sin necesidad de pre-segmentar por
    huecos de índice o de rango.

Pipeline:
    /scan → filtrar + separar en sector izq/der (excluyendo el frontal,
            reservado para detección de obstáculos) → RANSAC por lado
          → refinar por PCA sobre los inliers → distancia perpendicular
          → fallback a mínimo rango crudo si RANSAC no encuentra pared
          → suavizado temporal (EMA) → publicar

    Convención angular (igual que behavior_fsm): af = ángulo relativo al
    frente del LiDAR. af > 0 → izquierda, af < 0 → derecha.

Tópicos:
    /dist_izq, /dist_der                   (Float32, m) — para behavior_fsm
    /dbg/confianza_izq, /dbg/confianza_der  (Float32)    — ratio de inliers RANSAC
    /dbg/ancho_jiron_medido                 (Float32, m) — dist_izq + dist_der
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from behavior_fsm import percepcion as pc


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',   180.0)  # igual que behavior_fsm
        self.declare_parameter('sector_frontal_deg', 30.0)  # excluido del ajuste de pared
        self.declare_parameter('sector_lateral_lo',  60.0)  # ventana estrecha para el
        self.declare_parameter('sector_lateral_hi', 120.0)  # fallback (cerca de perpendicular)
        self.declare_parameter('sector_trasero_deg', 160.0)  # excluido del ajuste (detrás del robot)
        self.declare_parameter('rango_max',           3.5)  # m  ignora lecturas más lejanas

        self.declare_parameter('umbral_inlier_ransac', 0.03)  # m  tolerancia de inlier
        self.declare_parameter('iteraciones_ransac',      80)
        self.declare_parameter('min_inliers_ransac',      12)
        self.declare_parameter('min_puntos_lado',         15)  # puntos mínimos para intentar RANSAC
        self.declare_parameter('semilla_ransac',          -1)  # -1 = aleatorio real

        self.declare_parameter('ema_alpha', 0.5)  # suavizado temporal (1=sin suavizar)

        self.declare_parameter('ancho_jiron',     0.60)  # m  ancho esperado (chequeo de consistencia)
        self.declare_parameter('tol_ancho_jiron', 0.15)  # m  tolerancia antes de advertir

        self.front_rad = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector    = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo    = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi    = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.sector_trasero = math.radians(self.get_parameter('sector_trasero_deg').value)
        self.rango_max = self.get_parameter('rango_max').value

        self.umbral_inlier = self.get_parameter('umbral_inlier_ransac').value
        self.iteraciones   = int(self.get_parameter('iteraciones_ransac').value)
        self.min_inliers   = int(self.get_parameter('min_inliers_ransac').value)
        self.min_lado      = int(self.get_parameter('min_puntos_lado').value)
        semilla = int(self.get_parameter('semilla_ransac').value)
        self._rng = random.Random(semilla) if semilla >= 0 else random.Random()

        self.alpha = self.get_parameter('ema_alpha').value

        self.ancho_jiron     = self.get_parameter('ancho_jiron').value
        self.tol_ancho_jiron = self.get_parameter('tol_ancho_jiron').value

        # ── Estado (para EMA) ─────────────────────────────────────────────
        self._izq_prev = float('inf')
        self._der_prev = float('inf')

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos_scan = QoSProfile(depth=10)
        _qos_scan.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._cb_scan, _qos_scan)

        self.pub_izq  = self.create_publisher(Float32, '/dist_izq', 10)
        self.pub_der  = self.create_publisher(Float32, '/dist_der', 10)
        self.pub_conf_izq = self.create_publisher(Float32, '/dbg/confianza_izq', 10)
        self.pub_conf_der = self.create_publisher(Float32, '/dbg/confianza_der', 10)
        self.pub_ancho    = self.create_publisher(Float32, '/dbg/ancho_jiron_medido', 10)

        self.get_logger().info(
            f'wall_follower (RANSAC) listo  |  umbral_inlier={self.umbral_inlier} m'
            f'  min_inliers={self.min_inliers}  ancho_jiron={self.ancho_jiron} m')

    # ── Filtrado y separación por lado ───────────────────────────────────
    def _separar_lados(self, msg: LaserScan):
        """Separa el scan en puntos (x,y) de cada lado, más el mínimo rango
        crudo dentro de la ventana estrecha (fallback). Excluye el sector
        frontal (reservado para detección de obstáculos) y el sector trasero
        (más allá de sector_trasero_deg): justo tras un GIRO/RODEO, "detrás"
        del robot puede corresponder a un tramo de pared distinto del que
        se está siguiendo ahora, y contaminaría el ajuste."""
        izq_xy, der_xy = [], []
        fallback_izq, fallback_der = float('inf'), float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > min(msg.range_max, self.rango_max):
                continue

            raw = msg.angle_min + i * msg.angle_increment
            af  = math.atan2(math.sin(raw - self.front_rad),
                             math.cos(raw - self.front_rad))
            abs_af = abs(af)
            if abs_af <= self.sector or abs_af >= self.sector_trasero:
                continue  # frontal (obstáculos) o trasero (posible otra pared)

            x, y = pc.polar_a_cartesiano(raw, r)
            if af > 0:
                izq_xy.append((x, y))
                if self.lat_lo <= abs_af <= self.lat_hi:
                    fallback_izq = min(fallback_izq, r)
            else:
                der_xy.append((x, y))
                if self.lat_lo <= abs_af <= self.lat_hi:
                    fallback_der = min(fallback_der, r)

        return izq_xy, der_xy, fallback_izq, fallback_der

    # ── Distancia por lado: RANSAC con fallback a mínimo rango ──────────
    def _distancia_lado(self, puntos_xy, fallback):
        """Devuelve (distancia, confianza). confianza=0 si se usó el fallback."""
        if len(puntos_xy) >= self.min_lado:
            modelo = pc.ajustar_recta_ransac(
                puntos_xy,
                umbral_inlier=self.umbral_inlier,
                iteraciones=self.iteraciones,
                min_inliers=self.min_inliers,
                rng=self._rng)
            if modelo is not None:
                recta = (modelo['a'], modelo['b'], modelo['c'])
                return pc.distancia_recta_origen(recta), modelo['ratio']
        return fallback, 0.0

    @staticmethod
    def _suavizar(alpha, actual, previo):
        if not math.isfinite(previo) or not math.isfinite(actual):
            return actual
        return alpha * actual + (1.0 - alpha) * previo

    # ── Callback principal ────────────────────────────────────────────────
    def _cb_scan(self, msg: LaserScan):
        izq_xy, der_xy, fb_izq, fb_der = self._separar_lados(msg)

        d_izq, conf_izq = self._distancia_lado(izq_xy, fb_izq)
        d_der, conf_der = self._distancia_lado(der_xy, fb_der)

        d_izq = self._suavizar(self.alpha, d_izq, self._izq_prev)
        d_der = self._suavizar(self.alpha, d_der, self._der_prev)
        self._izq_prev, self._der_prev = d_izq, d_der

        if math.isfinite(d_izq) and math.isfinite(d_der):
            ancho_medido = d_izq + d_der
            if abs(ancho_medido - self.ancho_jiron) > self.tol_ancho_jiron:
                self.get_logger().warn(
                    f'Ancho de jirón inconsistente: medido={ancho_medido:.2f} m'
                    f' vs esperado={self.ancho_jiron:.2f} m'
                    f' (posible pared mal clasificada / caja confundida con pared)',
                    throttle_duration_sec=1.0)
            self._pub_f32(self.pub_ancho, ancho_medido)
        else:
            self._pub_f32(self.pub_ancho, float('nan'))

        self._pub_f32(self.pub_izq, d_izq)
        self._pub_f32(self.pub_der, d_der)
        self._pub_f32(self.pub_conf_izq, conf_izq)
        self._pub_f32(self.pub_conf_der, conf_der)

        self.get_logger().debug(
            f'izq={d_izq:.3f} (conf={conf_izq:.2f})  '
            f'der={d_der:.3f} (conf={conf_der:.2f})')

    @staticmethod
    def _pub_f32(pub, valor):
        m = Float32()
        m.data = float(valor)
        pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    nodo = WallFollower()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
