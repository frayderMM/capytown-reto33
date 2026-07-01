#!/usr/bin/env python3
"""
wall_follower.py  —  Deteccion de paredes laterales por minimos cuadrados.

Suscribe a /scan, ajusta una recta por regresion ortogonal (minimos
cuadrados, ver lidar_utils.recta_por_pca) a los puntos de cada lado
(izquierda / derecha) del jiron y publica la distancia perpendicular del
robot a cada pared en /dist_izq y /dist_der (std_msgs/Float32). behavior_fsm
consume estos dos topicos para el tracking lateral; el frente (parada de
seguridad) lo sigue midiendo behavior_fsm directamente sobre /scan, sin
saltos entre nodos, para no anadir latencia a esa reaccion critica.

Por que minimos cuadrados y no RANSAC:
    Se probo RANSAC (consenso robusto a outliers) pero en la practica dio
    peor resultado: el muestreo aleatorio produce variacion de cuadro a
    cuadro incluso con la pared quieta, lo que se nota como ruido en el
    control. Minimos cuadrados sobre TODOS los puntos del lado es
    deterministico (mismo scan → misma recta, siempre) y mas simple. La
    contrapartida es que ya no descarta outliers a proposito: si una caja
    queda pegada al costado (dentro del sector lateral, no en el frontal),
    sus puntos SI van a sesgar la recta de esa pared.

Pipeline:
    /scan → filtrar + separar en sector izq/der (excluyendo el frontal,
            reservado para deteccion de obstaculos, y el trasero) →
            recta_por_pca() por lado → distancia perpendicular
          → fallback a minimo rango crudo si no hay suficientes puntos
          → suavizado temporal (EMA) → publicar

    Convencion angular (igual que behavior_fsm): af = angulo relativo al
    frente del LiDAR. af > 0 → izquierda, af < 0 → derecha.

Topicos:
    /dist_izq, /dist_der              (Float32, m)   — para behavior_fsm
    /dbg/confianza_izq, /dbg/confianza_der (Float32) — 1.0 si se ajusto
        recta, 0.0 si se uso el fallback (minimo rango crudo)
    /dbg/ancho_jiron_medido           (Float32, m)   — dist_izq + dist_der
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from box_detector import lidar_utils as lu


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # ── Parametros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',   180.0)  # igual que behavior_fsm
        self.declare_parameter('sector_frontal_deg', 30.0)  # excluido del ajuste de pared
        self.declare_parameter('sector_lateral_lo',  60.0)  # ventana estrecha para el
        self.declare_parameter('sector_lateral_hi', 120.0)  # fallback (cerca de perpendicular)
        self.declare_parameter('sector_trasero_deg', 160.0)  # excluido del ajuste (detras del robot)
        self.declare_parameter('rango_max',           3.5)  # m  ignora lecturas mas lejanas

        self.declare_parameter('min_puntos_lado', 15)  # puntos minimos para ajustar la recta

        self.declare_parameter('ema_alpha', 0.5)  # suavizado temporal (1=sin suavizar)

        self.declare_parameter('ancho_jiron',     0.60)  # m  ancho esperado (chequeo de consistencia)
        self.declare_parameter('tol_ancho_jiron', 0.15)  # m  tolerancia antes de advertir

        self.front_rad = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector    = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo    = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi    = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.sector_trasero = math.radians(self.get_parameter('sector_trasero_deg').value)
        self.rango_max = self.get_parameter('rango_max').value

        self.min_lado = int(self.get_parameter('min_puntos_lado').value)

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
            f'wall_follower (minimos cuadrados) listo  |  min_puntos_lado={self.min_lado}'
            f'  ancho_jiron={self.ancho_jiron} m')

    # ── Filtrado y separacion por lado ───────────────────────────────────
    def _separar_lados(self, msg: LaserScan):
        """Separa el scan en puntos (x,y) de cada lado, mas el minimo rango
        crudo dentro de la ventana estrecha (fallback). Excluye el sector
        frontal (reservado para deteccion de obstaculos) y el sector trasero
        (mas alla de sector_trasero_deg): justo tras un GIRO/RODEO, "detras"
        del robot puede corresponder a un tramo de pared distinto del que
        se esta siguiendo ahora, y contaminaria el ajuste."""
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
                continue  # frontal (obstaculos) o trasero (posible otra pared)

            x, y = lu.polar_a_cartesiano(raw, r)
            if af > 0:
                izq_xy.append((x, y))
                if self.lat_lo <= abs_af <= self.lat_hi:
                    fallback_izq = min(fallback_izq, r)
            else:
                der_xy.append((x, y))
                if self.lat_lo <= abs_af <= self.lat_hi:
                    fallback_der = min(fallback_der, r)

        return izq_xy, der_xy, fallback_izq, fallback_der

    # ── Distancia por lado: minimos cuadrados con fallback a minimo rango ──
    def _distancia_lado(self, puntos_xy, fallback):
        """Devuelve (distancia, confianza). Ajusta una recta por minimos
        cuadrados a TODOS los puntos del lado (deterministico, sin
        muestreo aleatorio). confianza=1.0 si se pudo ajustar, 0.0 si se
        uso el fallback (minimo rango crudo) por falta de puntos."""
        if len(puntos_xy) >= self.min_lado:
            recta = lu.recta_por_pca(puntos_xy)
            return lu.distancia_recta_origen(recta), 1.0
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
                    f'Ancho de jiron inconsistente: medido={ancho_medido:.2f} m'
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
