#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Tres estados:
  CRUCERO → avanza pegado a la pared derecha.
             Detecta caja perpendicular (~20 cm) por std de profundidad + ancho.
  GIRO    → gira a la izquierda para rodear la caja.
  RODEO   → tras girar, avanza recto junto a la caja hasta que la pared
             derecha reaparece, luego vuelve a CRUCERO.

Deteccion de caja = mismo criterio que lidar_viz.py v7:
  std_x < perp_std_max  (superficie perpendicular al robot)
  box_w_min <= y_spread <= box_w_max  (ancho compatible con caja ~20 cm)

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
RODEO   = 'RODEO'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',     180.0)
        self.declare_parameter('sector_frontal_deg',   30.0)  # sector ancho emergencia
        self.declare_parameter('sector_lateral_lo',    60.0)
        self.declare_parameter('sector_lateral_hi',   120.0)

        # --- Deteccion de caja (mismo criterio que lidar_viz.py v7) ---
        self.declare_parameter('detect_half_deg',      20.0)
        self.declare_parameter('detect_max_r',          0.45)
        self.declare_parameter('perp_std_max',          0.04)
        self.declare_parameter('box_w_min',             0.08)
        self.declare_parameter('box_w_max',             0.23)
        self.declare_parameter('min_box_pts',           5)

        # --- Distancias ---
        self.declare_parameter('dist_alerta',           0.45)  # m frente libre
        self.declare_parameter('dist_pared_lateral',    0.55)  # m pared der reapareció
        self.declare_parameter('dist_emergencia',       0.12)  # m stop total

        # --- Velocidades ---
        self.declare_parameter('vel_crucero',           0.18)
        self.declare_parameter('vel_giro_gradual',      0.45)  # rad/s giro izquierda
        self.declare_parameter('vel_avance_giro',       0.08)  # m/s avance en GIRO

        # --- RODEO: avance recto para pasar junto a la caja ---
        self.declare_parameter('t_rodeo',               1.5)   # s tiempo en RODEO
        self.declare_parameter('v_rodeo',               0.15)  # m/s velocidad
        self.declare_parameter('w_rodeo',               0.0)   # rad/s (0=recto)

        # --- Temporizacion ---
        self.declare_parameter('t_giro_min',            0.5)
        self.declare_parameter('t_giro_max',            6.0)
        self.declare_parameter('t_cooldown',            1.5)   # s cooldown post-GIRO

        # --- Repulsion pared izquierda ---
        self.declare_parameter('dist_izq_min',          0.15)
        self.declare_parameter('Kizq',                  3.0)

        # ── Cargar valores ────────────────────────────────────────────────
        self.front_rad    = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector       = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo       = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi       = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.detect_half  = math.radians(self.get_parameter('detect_half_deg').value)
        self.detect_max_r = self.get_parameter('detect_max_r').value
        self.perp_std_max = self.get_parameter('perp_std_max').value
        self.box_w_min    = self.get_parameter('box_w_min').value
        self.box_w_max    = self.get_parameter('box_w_max').value
        self.min_box_pts  = self.get_parameter('min_box_pts').value
        self.d_alerta     = self.get_parameter('dist_alerta').value
        self.d_pared_lat  = self.get_parameter('dist_pared_lateral').value
        self.d_emerg      = self.get_parameter('dist_emergencia').value
        self.v_cruise     = self.get_parameter('vel_crucero').value
        self.w_giro       = self.get_parameter('vel_giro_gradual').value
        self.v_giro       = self.get_parameter('vel_avance_giro').value
        self.t_rodeo      = self.get_parameter('t_rodeo').value
        self.v_rodeo      = self.get_parameter('v_rodeo').value
        self.w_rodeo      = self.get_parameter('w_rodeo').value
        self.t_giro_min   = self.get_parameter('t_giro_min').value
        self.t_giro_max   = self.get_parameter('t_giro_max').value
        self.t_cooldown   = self.get_parameter('t_cooldown').value
        self.d_izq_min    = self.get_parameter('dist_izq_min').value
        self.Kizq         = self.get_parameter('Kizq').value

        # ── Estado ────────────────────────────────────────────────────────
        self.estado        = CRUCERO
        self.t_inicio      = self.get_clock().now()
        self.t_ultimo_giro = -float('inf')

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente  = float('inf')
        self.dist_izq     = float('inf')
        self.dist_der     = float('inf')
        self._w_lateral   = 0.0
        self.box_detected = False
        self.box_dist     = float('inf')

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan',               self.cb_scan, _qos)
        self.create_subscription(Float32,   '/lateral_correction', self._cb_lat,  10)
        self.pub_cmd    = self.create_publisher(Twist,   '/cmd_vel',     10)
        self.pub_estado = self.create_publisher(String,  '/fsm_state',   10)
        self.pub_parada = self.create_publisher(Float32, '/parada_dist', 10)
        self.create_timer(0.1, self.loop_control)

        self.get_logger().info('BehaviorFSM listo — CRUCERO / GIRO / RODEO')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def cb_scan(self, msg: LaserScan):
        d_f = d_l = d_r = float('inf')
        box_pts = []

        for i, r in enumerate(msg.ranges):
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            abs_af = abs(af)
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue

            if abs_af <= self.sector:
                d_f = min(d_f, r)

            if abs_af <= self.detect_half and r <= self.detect_max_r:
                box_pts.append((r * math.cos(af), r * math.sin(af)))

            if self.lat_lo <= abs_af <= self.lat_hi:
                if af > 0:
                    d_l = min(d_l, r)
                else:
                    d_r = min(d_r, r)

        self.dist_frente = d_f
        self.dist_izq    = d_l
        self.dist_der    = d_r

        # ── Deteccion de caja ─────────────────────────────────────────────
        self.box_detected = False
        self.box_dist     = float('inf')
        if len(box_pts) >= self.min_box_pts:
            xs    = [p[0] for p in box_pts]
            ys    = [p[1] for p in box_pts]
            n     = len(xs)
            mx    = sum(xs) / n
            std_x = math.sqrt(sum((x - mx) ** 2 for x in xs) / n)
            y_spread = max(ys) - min(ys)
            if std_x < self.perp_std_max and self.box_w_min <= y_spread <= self.box_w_max:
                self.box_detected = True
                self.box_dist     = mx

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
        if self.estado == GIRO and nuevo in (CRUCERO, RODEO):
            self.t_ultimo_giro = self.get_clock().now().nanoseconds * 1e-9
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    # ── FSM principal ─────────────────────────────────────────────────────
    def loop_control(self):

        # Emergencia — override de cualquier estado
        if self.dist_frente < self.d_emerg:
            self.get_logger().warn(
                f'EMERGENCIA frente={self.dist_frente:.2f}m', throttle_duration_sec=1.0)
            self._pub(0.0, 0.0)
            return

        # ── CRUCERO ───────────────────────────────────────────────────────
        if self.estado == CRUCERO:
            ahora = self.get_clock().now().nanoseconds * 1e-9
            cooldown_ok = (ahora - self.t_ultimo_giro) >= self.t_cooldown
            if cooldown_ok and self.box_detected:
                d_msg = Float32(); d_msg.data = float(self.box_dist)
                self.pub_parada.publish(d_msg)
                self._cambiar(GIRO)
                return

            v = self.v_cruise
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_izq_min:
                w = -self.Kizq * (self.d_izq_min - self.dist_izq)
            else:
                w = self._w_lateral if self.dist_frente >= self.d_alerta else 0.0
            self._pub(v, w)

        # ── GIRO ──────────────────────────────────────────────────────────
        elif self.estado == GIRO:
            # Seguridad: pared izquierda demasiado cerca → abortar giro
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_emerg:
                self._cambiar(RODEO)
                return
            if self._t_estado() > self.t_giro_max:
                self._cambiar(CRUCERO)
                return
            # Salida por TIEMPO FIJO: no usar dist_frente porque la pared izq
            # entra al cono frontal durante el giro y nunca dejaria salir.
            if self._t_estado() >= self.t_giro_min:
                if self.dist_der < self.d_pared_lat:
                    self._cambiar(CRUCERO)  # pared derecha reaparecio → saltar RODEO
                else:
                    self._cambiar(RODEO)    # frente libre, sin pared der → pasar caja recto
                return
            self._pub(self.v_giro, self.w_giro)

        # ── RODEO ─────────────────────────────────────────────────────────
        elif self.estado == RODEO:
            # Avanza recto hasta que la pared derecha reaparece o expira el tiempo
            if self._t_estado() > self.t_rodeo or self.dist_der < self.d_pared_lat:
                self._cambiar(CRUCERO)
                return
            self._pub(self.v_rodeo, self.w_rodeo)


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
