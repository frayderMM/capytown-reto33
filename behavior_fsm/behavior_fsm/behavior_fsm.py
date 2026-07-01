#!/usr/bin/env python3
"""
behavior_fsm.py  —  "El Guardian v2"

Logica:
  CRUCERO:
    w = w_der (tracking pared derecha directo)
      + w_front (correccion gradual frontal, desde d_front_inicio)
      + w_izq (repulsion pared izquierda)
    Si frente < d_giro → GIRO.

  GIRO:
    Gira izquierda hasta que frente despeje.

  STOP (override maxima prioridad):
    Solo si frente < d_stop_front (8 cm) o izq < d_stop_lateral (5 cm).
    NO para por la distancia normal de pared.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


CRUCERO = 'CRUCERO'
GIRO    = 'GIRO'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  30.0)
        self.declare_parameter('sector_lateral_lo',   60.0)
        self.declare_parameter('sector_lateral_hi',  120.0)

        # Zonas de crash (parada absoluta, muy pequeñas)
        self.declare_parameter('d_stop_front',   0.08)  # m  para si frente < 8 cm
        self.declare_parameter('d_stop_lateral', 0.05)  # m  para si izq < 5 cm

        # Umbrales de accion
        self.declare_parameter('d_giro',         0.22)  # m  trigger GIRO
        self.declare_parameter('d_front_inicio', 0.40)  # m  inicio correccion gradual frontal

        # Control pared derecha (objetivo 10-16 cm, target=13 cm)
        self.declare_parameter('target_der', 0.13)   # m  distancia objetivo
        self.declare_parameter('Kder',        4.0)   # ganancia

        # Repulsion pared izquierda
        self.declare_parameter('d_izq_min', 0.15)   # m  inicio repulsion
        self.declare_parameter('Kizq',       4.0)

        # Correccion frontal gradual
        self.declare_parameter('Kfront', 1.2)

        # Velocidad
        self.declare_parameter('vel_crucero',      0.10)  # m/s
        self.declare_parameter('vel_giro_gradual', 0.50)  # rad/s
        self.declare_parameter('max_w',            0.60)  # rad/s cap angular

        # Temporizacion
        self.declare_parameter('t_giro_min', 1.0)
        self.declare_parameter('t_giro_max', 4.0)
        self.declare_parameter('t_cooldown', 2.0)

        # ── Cargar ────────────────────────────────────────────────────────
        self.front_rad    = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector       = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo       = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi       = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.d_stop_front = self.get_parameter('d_stop_front').value
        self.d_stop_lat   = self.get_parameter('d_stop_lateral').value
        self.d_giro       = self.get_parameter('d_giro').value
        self.d_front_ini  = self.get_parameter('d_front_inicio').value
        self.target_der   = self.get_parameter('target_der').value
        self.Kder         = self.get_parameter('Kder').value
        self.d_izq_min    = self.get_parameter('d_izq_min').value
        self.Kizq         = self.get_parameter('Kizq').value
        self.Kfront       = self.get_parameter('Kfront').value
        self.v_cruise     = self.get_parameter('vel_crucero').value
        self.w_giro       = self.get_parameter('vel_giro_gradual').value
        self.max_w        = self.get_parameter('max_w').value
        self.t_giro_min   = self.get_parameter('t_giro_min').value
        self.t_giro_max   = self.get_parameter('t_giro_max').value
        self.t_cooldown   = self.get_parameter('t_cooldown').value

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
        self.create_timer(0.05, self.loop_control)  # 20 Hz (antes 10 Hz)

        self.get_logger().info(
            f'Guardian v2  stop_f<{self.d_stop_front}m  giro<{self.d_giro}m'
            f'  front_ini={self.d_front_ini}m  target_der={self.target_der}m'
            f'  Kder={self.Kder}  Kfront={self.Kfront}')

    # ── Scan ──────────────────────────────────────────────────────────────
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
            f'{self.estado}→{nuevo}  '
            f'f={self.dist_frente:.2f}  l={self.dist_izq:.2f}  r={self.dist_der:.2f}')
        if self.estado == GIRO and nuevo == CRUCERO:
            self.t_ultimo_giro = self.get_clock().now().nanoseconds * 1e-9
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    # ── FSM ───────────────────────────────────────────────────────────────
    def loop_control(self):

        # ── PRIORIDAD 1: ZONA DE CRASH → PARA ────────────────────────────
        if self.dist_frente < self.d_stop_front:
            self._pub(0.0, 0.0)
            self.get_logger().warn(
                f'PARA frente={self.dist_frente:.3f}m', throttle_duration_sec=0.4)
            return
        if math.isfinite(self.dist_izq) and self.dist_izq < self.d_stop_lat:
            self._pub(0.0, 0.0)
            self.get_logger().warn(
                f'PARA izq={self.dist_izq:.3f}m', throttle_duration_sec=0.4)
            return

        # ── CRUCERO ───────────────────────────────────────────────────────
        if self.estado == CRUCERO:
            ahora = self.get_clock().now().nanoseconds * 1e-9
            if (ahora - self.t_ultimo_giro) >= self.t_cooldown and \
               self.dist_frente <= self.d_giro:
                self._cambiar(GIRO)
                return

            # Correccion frontal gradual (gira izq mientras el frente se cierra)
            w_front = 0.0
            if self.dist_frente < self.d_front_ini:
                # proporcional: mas cerca → mas giro izquierda
                w_front = self.Kfront * (self.d_front_ini - self.dist_frente)

            # Tracking pared derecha directo (sin wall_follower)
            # w<0 = gira derecha (se acerca a pared), w>0 = gira izq (se aleja)
            w_der = 0.0
            if math.isfinite(self.dist_der):
                w_der = -self.Kder * (self.dist_der - self.target_der)

            # Repulsion pared izquierda
            w_izq = 0.0
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_izq_min:
                w_izq = -self.Kizq * (self.d_izq_min - self.dist_izq)

            w = max(-self.max_w, min(self.max_w, w_front + w_der + w_izq))
            self._pub(self.v_cruise, w)

        # ── GIRO ──────────────────────────────────────────────────────────
        elif self.estado == GIRO:
            if self._t_estado() > self.t_giro_max:
                self._cambiar(CRUCERO)
                return
            if self._t_estado() >= self.t_giro_min and self.dist_frente > self.d_giro:
                self._cambiar(CRUCERO)
                return
            self._pub(self.v_cruise, self.w_giro)


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
