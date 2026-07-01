#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Logica:
  1. SIEMPRE pegado a la pared derecha (wall_follower via /lateral_correction).
  2. Si hay obstaculo al frente < dist_giro  →  GIRO izquierda.
  3. Si supera limites (frente < dist_parar  o  izq < dist_parar) →  PARA.
  4. La parada es prioritaria sobre cualquier otra accion.

Estados:
  CRUCERO → avanza siguiendo pared derecha.
  GIRO    → gira a la izquierda hasta que el frente quede libre.
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
        self.declare_parameter('sector_frontal_deg',  30.0)
        self.declare_parameter('sector_lateral_lo',   60.0)
        self.declare_parameter('sector_lateral_hi',  120.0)

        # --- Umbrales de distancia ---
        self.declare_parameter('dist_parar',   0.20)  # m  PARA si frente o izq < esto
        self.declare_parameter('dist_giro',    0.35)  # m  GIRO si frente < esto
        self.declare_parameter('dist_izq_min', 0.10)  # m  repulsion izq si < esto

        # --- Velocidad ---
        self.declare_parameter('vel_crucero',      0.10)  # m/s  (misma en todos los estados)
        self.declare_parameter('vel_giro_gradual', 0.40)  # rad/s giro izquierda

        # --- Ganancia repulsion izquierda ---
        self.declare_parameter('Kizq', 4.0)

        # --- Temporizacion ---
        self.declare_parameter('t_giro_min', 1.2)   # s  minimo en GIRO
        self.declare_parameter('t_giro_max', 4.0)   # s  salvavidas
        self.declare_parameter('t_cooldown', 2.0)   # s  cooldown post-GIRO

        # ── Cargar valores ────────────────────────────────────────────────
        self.front_rad  = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector     = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo     = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi     = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.d_parar    = self.get_parameter('dist_parar').value
        self.d_giro     = self.get_parameter('dist_giro').value
        self.d_izq_min  = self.get_parameter('dist_izq_min').value
        self.v_cruise   = self.get_parameter('vel_crucero').value
        self.w_giro     = self.get_parameter('vel_giro_gradual').value
        self.Kizq       = self.get_parameter('Kizq').value
        self.t_giro_min = self.get_parameter('t_giro_min').value
        self.t_giro_max = self.get_parameter('t_giro_max').value
        self.t_cooldown = self.get_parameter('t_cooldown').value

        # ── Estado ────────────────────────────────────────────────────────
        self.estado        = CRUCERO
        self.t_inicio      = self.get_clock().now()
        self.t_ultimo_giro = -float('inf')

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente = float('inf')
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self._w_lateral  = 0.0

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan',               self.cb_scan, _qos)
        self.create_subscription(Float32,   '/lateral_correction', self._cb_lat, 10)
        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel',    10)
        self.pub_estado = self.create_publisher(String, '/fsm_state',  10)
        self.create_timer(0.1, self.loop_control)

        self.get_logger().info(
            f'BehaviorFSM listo  |  parar<{self.d_parar}m  giro<{self.d_giro}m'
            f'  izq_min={self.d_izq_min}m  v={self.v_cruise}m/s')

    # ── Callbacks ─────────────────────────────────────────────────────────
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
            f'frente={self.dist_frente:.2f}  izq={self.dist_izq:.2f}  der={self.dist_der:.2f}')
        if self.estado == GIRO and nuevo == CRUCERO:
            self.t_ultimo_giro = self.get_clock().now().nanoseconds * 1e-9
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def _limite_superado(self) -> bool:
        """True si cualquier sensor supera el limite de seguridad → PARA."""
        if self.dist_frente < self.d_parar:
            return True
        if math.isfinite(self.dist_izq) and self.dist_izq < self.d_parar:
            return True
        return False

    # ── FSM principal ─────────────────────────────────────────────────────
    def loop_control(self):

        # ── PRIORIDAD 1: PARADA DE SEGURIDAD ─────────────────────────────
        if self._limite_superado():
            self.get_logger().warn(
                f'PARADA  frente={self.dist_frente:.2f}  izq={self.dist_izq:.2f}',
                throttle_duration_sec=0.5)
            self._pub(0.0, 0.0)
            return

        # ── CRUCERO ───────────────────────────────────────────────────────
        if self.estado == CRUCERO:
            ahora = self.get_clock().now().nanoseconds * 1e-9
            cooldown_ok = (ahora - self.t_ultimo_giro) >= self.t_cooldown

            if cooldown_ok and self.dist_frente <= self.d_giro:
                self._cambiar(GIRO)
                return

            # Siempre hacia la derecha: wall_follower o repulsion izq
            v = self.v_cruise
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_izq_min:
                w = -self.Kizq * (self.d_izq_min - self.dist_izq)
            else:
                w = self._w_lateral
            self._pub(v, w)

        # ── GIRO ──────────────────────────────────────────────────────────
        elif self.estado == GIRO:
            if self._t_estado() > self.t_giro_max:
                self._cambiar(CRUCERO)
                return
            # Sale cuando el frente queda libre Y ha girado el minimo
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
