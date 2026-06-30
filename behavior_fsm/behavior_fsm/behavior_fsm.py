#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Logica reactiva simple, dos estados:
  CRUCERO → avanza pegado a la pared derecha (correccion de wall_follower),
             con velocidad proporcional a la distancia frontal.
  GIRO    → al detectar un obstaculo (caja/pared) a dist_obstaculo, gira
             a la izquierda con velocidad angular fija mientras avanza
             despacio (giro amplio, no en el sitio), hasta que la pared
             derecha vuelve a aparecer y el frente esta libre.

Sin busqueda de hueco ni sub-estados de rescate: el sentido de giro es
siempre el mismo (izquierda), lo que evita el bucle de re-evaluacion que
producia vueltas en circulos en las esquinas.

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
GIRO    = 'GIRO'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  45.0)
        self.declare_parameter('sector_lateral_lo',   60.0)
        self.declare_parameter('sector_lateral_hi',  120.0)

        # --- Distancias de reaccion ---
        self.declare_parameter('dist_alerta',          0.55)  # m  empieza a frenar progresivamente
        self.declare_parameter('dist_obstaculo',        0.40)  # m  dispara el giro (caja/pared al frente)
        self.declare_parameter('dist_pared_lateral',     0.55)  # m  umbral para considerar "pared der detectada"
        self.declare_parameter('dist_emergencia',        0.12)  # m  stop total override

        # --- Velocidades ---
        self.declare_parameter('vel_crucero',          0.18)
        self.declare_parameter('vel_min',              0.05)
        self.declare_parameter('vel_giro_gradual',      0.35)  # rad/s  giro fijo a la izquierda
        self.declare_parameter('vel_avance_giro',       0.08)  # m/s  avance lento mientras gira

        # --- Temporizacion del giro ---
        self.declare_parameter('t_giro_min',            0.5)   # s  minimo en GIRO antes de poder salir
        self.declare_parameter('t_giro_max',            6.0)   # s  salvavidas: vuelve a CRUCERO igual

        self.front_rad  = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector     = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo     = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi     = math.radians(self.get_parameter('sector_lateral_hi').value)

        self.d_alerta   = self.get_parameter('dist_alerta').value
        self.d_obst     = self.get_parameter('dist_obstaculo').value
        self.d_pared_lat = self.get_parameter('dist_pared_lateral').value
        self.d_emerg    = self.get_parameter('dist_emergencia').value

        self.v_cruise   = self.get_parameter('vel_crucero').value
        self.v_min      = self.get_parameter('vel_min').value
        self.w_giro     = self.get_parameter('vel_giro_gradual').value
        self.v_giro     = self.get_parameter('vel_avance_giro').value

        self.t_giro_min = self.get_parameter('t_giro_min').value
        self.t_giro_max = self.get_parameter('t_giro_max').value

        # ── Estado ────────────────────────────────────────────────────────
        self.estado   = CRUCERO
        self.t_inicio = self.get_clock().now()

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente = float('inf')
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
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

        self.get_logger().info('BehaviorFSM listo — CRUCERO/GIRO (giro izquierdo fijo)')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def cb_scan(self, msg: LaserScan):
        d_f = d_l = d_r = float('inf')

        for i, r in enumerate(msg.ranges):
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            abs_af = abs(af)
            valid  = math.isfinite(r) and msg.range_min <= r <= msg.range_max
            if not valid:
                continue

            if abs_af <= self.sector:
                d_f = min(d_f, r)

            if self.lat_lo <= abs_af <= self.lat_hi:
                if af > 0:
                    d_l = min(d_l, r)
                else:
                    d_r = min(d_r, r)

        self.dist_frente = d_f
        self.dist_izq    = d_l
        self.dist_der    = d_r

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
            f'(frente={self.dist_frente:.2f} m  der={self.dist_der:.2f} m)')
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def _vel_adaptativa(self) -> float:
        d = self.dist_frente
        if d >= self.d_alerta:
            return self.v_cruise
        ratio = (d - self.d_obst) / (self.d_alerta - self.d_obst)
        return self.v_min + max(0.0, min(1.0, ratio)) * (self.v_cruise - self.v_min)

    # ── FSM principal ─────────────────────────────────────────────────────
    def loop_control(self):

        # Contingencia de colisión — override de cualquier estado
        if self.dist_frente < self.d_emerg:
            self.get_logger().warn(
                f'EMERGENCIA frente={self.dist_frente:.2f}m — stop total', throttle_duration_sec=1.0)
            self._pub(0.0, 0.0)
            return

        if self.estado == CRUCERO:
            if self.dist_frente <= self.d_obst:
                d_msg = Float32(); d_msg.data = float(self.dist_frente)
                self.pub_parada.publish(d_msg)
                self._cambiar(GIRO)
                return
            v = self._vel_adaptativa()
            w = self._w_lateral if self.dist_frente >= self.d_alerta else 0.0
            self._pub(v, w)

        elif self.estado == GIRO:
            # Salvavidas: si tarda demasiado (zona abierta sin pared der),
            # no se queda girando para siempre.
            if self._t_estado() > self.t_giro_max:
                self._cambiar(CRUCERO)
                return

            # Salida: frente libre Y la pared derecha reaparecio → pegarse de nuevo.
            if (self._t_estado() > self.t_giro_min
                    and self.dist_frente > self.d_obst
                    and self.dist_der < self.d_pared_lat):
                self._cambiar(CRUCERO)
                return

            self._pub(self.v_giro, self.w_giro)


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
