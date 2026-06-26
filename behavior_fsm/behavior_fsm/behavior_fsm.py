#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Gap navigation reactivo:
  CRUCERO → avanza con velocidad proporcional a la distancia frontal.
  PARAR   → para 0.3 s al detectar obstaculo.
  EVADIR  → gira dinamicamente hacia el angulo con mayor espacio abierto
             (donde el LiDAR no ve puntos, o los ve muy lejos).
             Cuando el frente queda libre, vuelve a CRUCERO.

No hay angulo fijo ni tiempo fijo de rodeo.
El robot sigue el hueco hasta salir del obstaculo.

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


CRUCERO = 'CRUCERO'
PARAR   = 'PARAR'
EVADIR  = 'EVADIR'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  45.0)
        self.declare_parameter('sector_lateral_lo',   60.0)
        self.declare_parameter('sector_lateral_hi',  120.0)
        self.declare_parameter('dist_alerta',          0.45)
        self.declare_parameter('dist_parada',          0.18)
        self.declare_parameter('vel_crucero',          0.18)
        self.declare_parameter('vel_min',              0.05)
        self.declare_parameter('vel_giro',             0.50)
        self.declare_parameter('pausa_parada',         0.30)
        self.declare_parameter('Kgap',                 1.2)   # rad/s por radian de gap
        self.declare_parameter('gap_sector_deg',       90.0)  # ventana de busqueda del hueco
        self.declare_parameter('t_evasion_max',        8.0)   # timeout de seguridad

        self.front_rad  = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector     = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo     = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi     = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.d_alerta   = self.get_parameter('dist_alerta').value
        self.d_parada   = self.get_parameter('dist_parada').value
        self.v_cruise   = self.get_parameter('vel_crucero').value
        self.v_min      = self.get_parameter('vel_min').value
        self.w_giro     = self.get_parameter('vel_giro').value
        self.t_pausa    = self.get_parameter('pausa_parada').value
        self.Kgap       = self.get_parameter('Kgap').value
        self.gap_sector = math.radians(self.get_parameter('gap_sector_deg').value)
        self.t_ev_max   = self.get_parameter('t_evasion_max').value

        # ── Estado ────────────────────────────────────────────────────────
        self.estado   = CRUCERO
        self.t_inicio = self.get_clock().now()

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente = float('inf')
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self._gap_ang    = 0.0   # ángulo al mayor espacio abierto
        self._w_lateral  = 0.0

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan',               self.cb_scan, _qos)
        self.create_subscription(Float32,   '/lateral_correction', self._cb_lat,  10)
        self.pub_cmd    = self.create_publisher(Twist,   '/cmd_vel',     10)
        self.pub_estado = self.create_publisher(String,  '/fsm_state',   10)
        self.pub_parada = self.create_publisher(Float32, '/parada_dist', 10)
        self.create_timer(0.1, self.loop_control)

        self.get_logger().info('BehaviorFSM listo — gap navigation reactivo')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def cb_scan(self, msg: LaserScan):
        d_f = d_l = d_r = float('inf')

        # Para el hueco: acumular rangos por bucket de 10°
        bucket_sum   = {}
        bucket_count = {}

        for i, r in enumerate(msg.ranges):
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            abs_af = abs(af)
            valid  = math.isfinite(r) and msg.range_min <= r <= msg.range_max

            # Sector frontal → dist_frente
            if abs_af <= self.sector and valid:
                d_f = min(d_f, r)

            # Sectores laterales → dist_izq / dist_der
            if self.lat_lo <= abs_af <= self.lat_hi and valid:
                if af > 0:
                    d_l = min(d_l, r)
                else:
                    d_r = min(d_r, r)

            # Ventana de búsqueda del hueco (±gap_sector)
            if abs_af <= self.gap_sector:
                # Sin retorno LiDAR = espacio abierto → usar range_max
                r_gap  = r if valid else msg.range_max
                bucket = round(math.degrees(af) / 10.0) * 10
                bucket_sum[bucket]   = bucket_sum.get(bucket, 0.0) + r_gap
                bucket_count[bucket] = bucket_count.get(bucket, 0) + 1

        self.dist_frente = d_f
        self.dist_izq    = d_l
        self.dist_der    = d_r

        # Hueco = bucket con mayor rango promedio dentro de la ventana
        if bucket_count:
            best = max(bucket_sum, key=lambda k: bucket_sum[k] / bucket_count[k])
            self._gap_ang = math.radians(best)

    def _cb_lat(self, msg: Float32):
        self._w_lateral = msg.data

    # ── Helpers ───────────────────────────────────────────────────────────
    def _pub(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String(); s.data = self.estado
        self.pub_estado.publish(s)

    def _t_estado(self) -> float:
        return (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9

    def _cambiar(self, nuevo: str):
        self.get_logger().info(
            f'{self.estado} → {nuevo}  '
            f'(frente={self.dist_frente:.2f} m  '
            f'gap={math.degrees(self._gap_ang):.0f}°)')
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def _vel_adaptativa(self) -> float:
        d = self.dist_frente
        if d >= self.d_alerta:
            return self.v_cruise
        ratio = (d - self.d_parada) / (self.d_alerta - self.d_parada)
        return self.v_min + max(0.0, min(1.0, ratio)) * (self.v_cruise - self.v_min)

    # ── FSM principal ─────────────────────────────────────────────────────
    def loop_control(self):

        if self.estado == CRUCERO:
            if self.dist_frente <= self.d_parada:
                d_msg = Float32(); d_msg.data = float(self.dist_frente)
                self.pub_parada.publish(d_msg)
                self._cambiar(PARAR)
                return
            v = self._vel_adaptativa()
            w = self._w_lateral if self.dist_frente >= self.d_alerta else 0.0
            self._pub(v, w)

        elif self.estado == PARAR:
            self._pub(0.0, 0.0)
            if self._t_estado() >= self.t_pausa:
                self._cambiar(EVADIR)

        elif self.estado == EVADIR:
            # Timeout de seguridad — evitar quedarse bloqueado para siempre
            if self._t_estado() > self.t_ev_max:
                self.get_logger().warn('Timeout evasion — forzando CRUCERO')
                self._cambiar(CRUCERO)
                return

            # Frente despejado → evasión completada
            if self.dist_frente > self.d_alerta:
                self._cambiar(CRUCERO)
                return

            gap = self._gap_ang
            w   = max(-self.w_giro, min(self.w_giro, self.Kgap * gap))

            # Avanzar lento solo si el hueco está casi al frente Y no hay pared cerca
            puede_avanzar = (
                abs(gap) < math.radians(25.0)
                and self.dist_frente > self.d_parada
            )
            v = self.v_min if puede_avanzar else 0.0

            self._pub(v, w)


def main(args=None):
    rclpy.init(args=args)
    nodo = BehaviorFSM()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo._pub(0.0, 0.0)
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
