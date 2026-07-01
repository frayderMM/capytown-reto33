#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Wall-following por metodo de DOS RAYOS (F1Tenth / MIT):
  - r90: rango a 90° del frente (perpendicular a pared derecha)
  - r45: rango a 45° del frente (diagonal derecha-adelante)
  Con geometria trigonometrica se calcula:
    theta     = angulo de la pared respecto al robot (0 = paralelo)
    d_pared   = distancia perpendicular real a la pared
  Control:
    w = -Kp*(d_pared - objetivo) - Ka*theta
  Sin deteccion de segmentos, sin longitud minima — siempre da un valor.

Estados:
  CRUCERO → avanza con wall-following dos rayos.
  GIRO    → gira izquierda tiempo fijo al detectar caja/pared.
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

ALPHA = math.pi / 4   # angulo entre los dos rayos (45°)


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',     180.0)
        self.declare_parameter('sector_frontal_deg',   30.0)
        self.declare_parameter('sector_lateral_lo',    60.0)
        self.declare_parameter('sector_lateral_hi',   120.0)

        # --- Deteccion de caja ---
        self.declare_parameter('detect_half_deg',      20.0)
        self.declare_parameter('detect_max_r',          0.45)
        self.declare_parameter('perp_std_max',          0.04)
        self.declare_parameter('box_w_min',             0.08)
        self.declare_parameter('box_w_max',             0.23)
        self.declare_parameter('min_box_pts',           5)

        # --- Distancias ---
        self.declare_parameter('dist_pared_lateral',    0.55)
        self.declare_parameter('dist_emergencia',       0.12)

        # --- Velocidad unica ---
        self.declare_parameter('vel_crucero',           0.14)
        self.declare_parameter('vel_giro_gradual',      0.40)  # rad/s

        # --- Wall-following dos rayos (derecha) ---
        self.declare_parameter('d_objetivo_der',        0.08)  # m objetivo
        self.declare_parameter('Kp_der',                1.0)   # ganancia distancia
        self.declare_parameter('Ka_der',                0.5)   # ganancia angulo
        self.declare_parameter('max_w_der',             0.40)  # rad/s tope

        # --- Repulsion pared izquierda ---
        self.declare_parameter('dist_izq_min',          0.15)
        self.declare_parameter('Kizq',                  3.0)

        # --- Temporizacion ---
        self.declare_parameter('t_giro_min',            1.2)
        self.declare_parameter('t_giro_max',            4.0)
        self.declare_parameter('t_cooldown',            3.0)

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
        self.d_pared_lat  = self.get_parameter('dist_pared_lateral').value
        self.d_emerg      = self.get_parameter('dist_emergencia').value
        self.v_cruise     = self.get_parameter('vel_crucero').value
        self.w_giro       = self.get_parameter('vel_giro_gradual').value
        self.d_obj_der    = self.get_parameter('d_objetivo_der').value
        self.Kp_der       = self.get_parameter('Kp_der').value
        self.Ka_der       = self.get_parameter('Ka_der').value
        self.max_w_der    = self.get_parameter('max_w_der').value
        self.d_izq_min    = self.get_parameter('dist_izq_min').value
        self.Kizq         = self.get_parameter('Kizq').value
        self.t_giro_min   = self.get_parameter('t_giro_min').value
        self.t_giro_max   = self.get_parameter('t_giro_max').value
        self.t_cooldown   = self.get_parameter('t_cooldown').value

        # Angulos raw de los dos rayos (con front_rad = 180°)
        # rayo 90°: perpendicular derecha → af = -π/2 → raw = front_rad - π/2
        # rayo 45°: diagonal derecha-fwd  → af = -π/4 → raw = front_rad - π/4
        self._raw_90 = self.front_rad - math.pi / 2
        self._raw_45 = self.front_rad - math.pi / 4

        # ── Estado ────────────────────────────────────────────────────────
        self.estado        = CRUCERO
        self.t_inicio      = self.get_clock().now()
        self.t_ultimo_giro = -float('inf')

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente  = float('inf')
        self.dist_izq     = float('inf')
        self.dist_der     = float('inf')
        self.r90          = float('inf')   # rayo perpendicular derecha
        self.r45          = float('inf')   # rayo 45° derecha-fwd
        self.box_detected = False
        self.box_dist     = float('inf')

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.cb_scan, _qos)
        self.pub_cmd    = self.create_publisher(Twist,   '/cmd_vel',     10)
        self.pub_estado = self.create_publisher(String,  '/fsm_state',   10)
        self.pub_parada = self.create_publisher(Float32, '/parada_dist', 10)
        self.create_timer(0.1, self.loop_control)

        self.get_logger().info('BehaviorFSM listo — wall-following DOS RAYOS')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _range_at(self, msg: LaserScan, raw_target: float) -> float:
        """Rango valido mas cercano al angulo raw_target. Promedia ±3 indices."""
        idx0 = round((raw_target - msg.angle_min) / msg.angle_increment)
        vals = []
        for d in range(-3, 4):
            i = idx0 + d
            if 0 <= i < len(msg.ranges):
                r = msg.ranges[i]
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    vals.append(r)
        if not vals:
            return float('inf')
        return sum(vals) / len(vals)

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

        # Dos rayos para wall-following
        self.r90 = self._range_at(msg, self._raw_90)
        self.r45 = self._range_at(msg, self._raw_45)

        # Deteccion de caja
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
            f'(r90={self.r90:.2f}  r45={self.r45:.2f}  der={self.dist_der:.2f})')
        if self.estado == GIRO and nuevo == CRUCERO:
            self.t_ultimo_giro = self.get_clock().now().nanoseconds * 1e-9
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def _w_dos_rayos(self) -> float:
        """
        Correccion angular por metodo de dos rayos (F1Tenth).
          theta   = angulo pared respecto robot (0 = paralelo)
          d_pared = distancia perpendicular real
          w = -Kp*(d_pared - obj) - Ka*theta
        """
        r90 = self.r90
        r45 = self.r45
        if not (math.isfinite(r90) and math.isfinite(r45)):
            return 0.0
        theta  = math.atan2(r45 * math.cos(ALPHA) - r90,
                            r45 * math.sin(ALPHA))
        d_wall = r90 * math.cos(theta)
        err_d  = d_wall - self.d_obj_der   # + = demasiado lejos
        w      = -self.Kp_der * err_d - self.Ka_der * theta
        return max(-self.max_w_der, min(self.max_w_der, w))

    # ── FSM principal ─────────────────────────────────────────────────────
    def loop_control(self):

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
                # Repulsion pared izquierda (prioridad)
                w = -self.Kizq * (self.d_izq_min - self.dist_izq)
            else:
                w = self._w_dos_rayos()
            self._pub(v, w)

        # ── GIRO ──────────────────────────────────────────────────────────
        elif self.estado == GIRO:
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_emerg:
                self._cambiar(CRUCERO)
                return
            if self._t_estado() > self.t_giro_max:
                self._cambiar(CRUCERO)
                return
            if self._t_estado() >= self.t_giro_min:
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
