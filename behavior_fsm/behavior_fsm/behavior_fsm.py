#!/usr/bin/env python3
"""
behavior_fsm.py  —  Guardian v3

Estados: CRUCERO → GIRO → RODEO → CRUCERO

CRUCERO:  sigue pared derecha.
          f >= d_front_ini → solo w_der (tracking pared)
          f <  d_front_ini → solo w_front (evasion frontal, w_der=0)
          f <= d_giro      → pasa a GIRO
GIRO:     gira izquierda fijo.
          Sale si: izq<d_izq_salida | frente despeja tras t_giro_min | max t_giro_max
RODEO:    avanza RECTO (w=0) durante t_rodeo segundos.
          Crea separacion fisica del obstaculo antes de retomar tracking.
CRUCERO(recovery): durante t_recuperacion, solo w_der sin w_front.

Topicos debug: /dist_frente /dist_izq /dist_der /dbg/w_front /dbg/w_der /dbg/w_izq /dbg/w_total
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

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
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  30.0)
        self.declare_parameter('sector_lateral_lo',   60.0)
        self.declare_parameter('sector_lateral_hi',  120.0)

        self.declare_parameter('d_stop_front',   0.14)
        self.declare_parameter('d_stop_lateral', 0.06)
        self.declare_parameter('d_giro',         0.30)
        self.declare_parameter('d_front_inicio', 0.40)

        self.declare_parameter('target_der', 0.17)   # robot consistente a 17-20cm
        self.declare_parameter('Kder',        2.0)   # suave: radio giro = 0.10/0.14 ≈ 70cm
        self.declare_parameter('d_izq_min',  0.15)
        self.declare_parameter('Kizq',        4.0)
        self.declare_parameter('Kfront',      2.0)

        self.declare_parameter('vel_crucero',      0.10)
        self.declare_parameter('vel_giro_gradual', 0.50)
        self.declare_parameter('max_w',            0.60)

        self.declare_parameter('t_giro_min',        0.8)
        self.declare_parameter('t_giro_max',        4.0)
        self.declare_parameter('d_izq_salida_giro', 0.20)
        self.declare_parameter('t_rodeo',           1.0)   # recto tras GIRO
        self.declare_parameter('t_cooldown',        2.0)
        self.declare_parameter('t_recuperacion',    1.5)

        # ── Cargar ────────────────────────────────────────────────────────
        self.front_rad = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector    = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo    = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi    = math.radians(self.get_parameter('sector_lateral_hi').value)
        self._reload_params()
        self.add_on_set_parameters_callback(self._on_params)

        # ── Estado ────────────────────────────────────────────────────────
        self.estado        = CRUCERO
        self.t_inicio      = self.get_clock().now()
        self.t_ultimo_giro = -float('inf')

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente = float('inf')
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.cb_scan, _qos)
        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel',   10)
        self.pub_estado = self.create_publisher(String, '/fsm_state', 10)

        self.pub_df  = self.create_publisher(Float32, '/dist_frente',  10)
        self.pub_dl  = self.create_publisher(Float32, '/dist_izq',     10)
        self.pub_dr  = self.create_publisher(Float32, '/dist_der',     10)
        self.pub_wf  = self.create_publisher(Float32, '/dbg/w_front',  10)
        self.pub_wd  = self.create_publisher(Float32, '/dbg/w_der',    10)
        self.pub_wi  = self.create_publisher(Float32, '/dbg/w_izq',    10)
        self.pub_wt  = self.create_publisher(Float32, '/dbg/w_total',  10)

        self.create_timer(0.05, self.loop_control)

        self.get_logger().info(
            f'Guardian v3  giro<{self.d_giro}m  front_ini={self.d_front_ini}m'
            f'  target_der={self.target_der}m  Kder={self.Kder}'
            f'  t_rodeo={self.t_rodeo}s')

    def _reload_params(self):
        self.d_stop_front      = self.get_parameter('d_stop_front').value
        self.d_stop_lat        = self.get_parameter('d_stop_lateral').value
        self.d_giro            = self.get_parameter('d_giro').value
        self.d_front_ini       = self.get_parameter('d_front_inicio').value
        self.target_der        = self.get_parameter('target_der').value
        self.Kder              = self.get_parameter('Kder').value
        self.d_izq_min         = self.get_parameter('d_izq_min').value
        self.Kizq              = self.get_parameter('Kizq').value
        self.Kfront            = self.get_parameter('Kfront').value
        self.v_cruise          = self.get_parameter('vel_crucero').value
        self.w_giro            = self.get_parameter('vel_giro_gradual').value
        self.max_w             = self.get_parameter('max_w').value
        self.t_giro_min        = self.get_parameter('t_giro_min').value
        self.t_giro_max        = self.get_parameter('t_giro_max').value
        self.d_izq_salida_giro = self.get_parameter('d_izq_salida_giro').value
        self.t_rodeo           = self.get_parameter('t_rodeo').value
        self.t_cooldown        = self.get_parameter('t_cooldown').value
        self.t_recuperacion    = self.get_parameter('t_recuperacion').value

    def _on_params(self, params):
        self._reload_params()
        self.get_logger().info(f'Params: {[p.name for p in params]}')
        return SetParametersResult(successful=True)

    def cb_scan(self, msg: LaserScan):
        d_f = d_l = d_r = float('inf')
        for i, r in enumerate(msg.ranges):
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            abs_af = abs(af)
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
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

    def _pub(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String(); s.data = self.estado
        self.pub_estado.publish(s)

    def _pub_dbg(self, wf, wd, wi, wt):
        def f32(x):
            m = Float32()
            m.data = float(x) if math.isfinite(x) else -999.0
            return m
        self.pub_df.publish(f32(self.dist_frente))
        self.pub_dl.publish(f32(self.dist_izq))
        self.pub_dr.publish(f32(self.dist_der))
        self.pub_wf.publish(f32(wf))
        self.pub_wd.publish(f32(wd))
        self.pub_wi.publish(f32(wi))
        self.pub_wt.publish(f32(wt))

    def _t_estado(self) -> float:
        return (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9

    def _cambiar(self, nuevo: str):
        self.get_logger().info(
            f'{self.estado}→{nuevo}  '
            f'f={self.dist_frente:.2f}  l={self.dist_izq:.2f}  r={self.dist_der:.2f}')
        # Marca cooldown al salir de GIRO o RODEO hacia CRUCERO
        if self.estado in (GIRO, RODEO) and nuevo == CRUCERO:
            self.t_ultimo_giro = self.get_clock().now().nanoseconds * 1e-9
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def loop_control(self):

        # ── PRIORIDAD 1: STOP absoluto ────────────────────────────────────
        if self.dist_frente < self.d_stop_front:
            self._pub(0.0, 0.0)
            self._pub_dbg(0, 0, 0, 0)
            self.get_logger().warn(
                f'PARA frente={self.dist_frente:.3f}m', throttle_duration_sec=0.4)
            return
        if math.isfinite(self.dist_izq) and self.dist_izq < self.d_stop_lat:
            self._pub(0.0, 0.0)
            self._pub_dbg(0, 0, 0, 0)
            self.get_logger().warn(
                f'PARA izq={self.dist_izq:.3f}m', throttle_duration_sec=0.4)
            return

        # ── CRUCERO ───────────────────────────────────────────────────────
        if self.estado == CRUCERO:
            ahora = self.get_clock().now().nanoseconds * 1e-9
            t_post = ahora - self.t_ultimo_giro
            cooldown_ok = t_post >= self.t_cooldown

            if cooldown_ok and self.dist_frente <= self.d_giro:
                self._cambiar(GIRO)
                self._pub_dbg(0, 0, 0, 0)
                return

            recuperando = t_post < self.t_recuperacion
            w_front = 0.0
            w_der   = 0.0

            if recuperando:
                # Solo tracking pared derecha — permite alinearse tras RODEO
                if math.isfinite(self.dist_der):
                    w_der = -self.Kder * (self.dist_der - self.target_der)
            elif self.dist_frente < self.d_front_ini:
                # Evasion frontal pura — sin competencia con w_der
                w_front = self.Kfront * (self.d_front_ini - self.dist_frente)
            else:
                # Tracking normal pared derecha
                if math.isfinite(self.dist_der):
                    w_der = -self.Kder * (self.dist_der - self.target_der)

            # Repulsion izquierda (siempre activa)
            w_izq = 0.0
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_izq_min:
                w_izq = -self.Kizq * (self.d_izq_min - self.dist_izq)

            w = max(-self.max_w, min(self.max_w, w_front + w_der + w_izq))
            self._pub_dbg(w_front, w_der, w_izq, w)
            self._pub(self.v_cruise, w)

        # ── GIRO ──────────────────────────────────────────────────────────
        elif self.estado == GIRO:
            if self._t_estado() > self.t_giro_max:
                self._cambiar(RODEO)
                return
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_izq_salida_giro:
                self.get_logger().warn(f'GIRO→RODEO izq={self.dist_izq:.2f}m')
                self._cambiar(RODEO)
                return
            if self._t_estado() >= self.t_giro_min and self.dist_frente > self.d_giro:
                self._cambiar(RODEO)
                return
            self._pub_dbg(self.w_giro, 0, 0, self.w_giro)
            self._pub(self.v_cruise, self.w_giro)

        # ── RODEO: avance recto para separarse del obstaculo ──────────────
        elif self.estado == RODEO:
            if self._t_estado() >= self.t_rodeo:
                self._cambiar(CRUCERO)
                return
            self._pub_dbg(0, 0, 0, 0)
            self._pub(self.v_cruise, 0.0)   # w=0: absolutamente recto


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
