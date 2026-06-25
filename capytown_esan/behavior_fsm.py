"""
behavior_fsm.py — RC3 (restructurado)

CRUCERO : avanza + centrado proporcional entre paredes
  - frente < 30cm SIN caja → GIRAR (gira derecha hasta frente libre)
  - frente < 50cm CON caja → CAJA_DETECTADA → PARAR → ESPERAR_3S → RODEAR

LiDAR: 0° = atras del robot, ±180° = frente del robot
  theta positivo (~+90°) = pared izquierda del jirón
  theta negativo (~-90°) = pared derecha del jirón
"""
import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseArray


class State(Enum):
    CRUCERO        = auto()
    GIRAR          = auto()
    CAJA_DETECTADA = auto()
    PARAR          = auto()
    ESPERAR_3S     = auto()
    RODEAR         = auto()


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        self.declare_parameter('cruise_speed',    0.22)
        self.declare_parameter('center_kp',       0.8)
        self.declare_parameter('corner_distance', 0.30)
        self.declare_parameter('clear_distance',  0.55)
        self.declare_parameter('alert_distance',  0.50)
        self.declare_parameter('alert_angle_deg', 40.0)
        self.declare_parameter('side_lo_deg',     60.0)
        self.declare_parameter('side_hi_deg',    120.0)
        self.declare_parameter('turn_speed',      0.45)
        self.declare_parameter('bypass_angle_deg',18.0)
        self.declare_parameter('bypass_forward',  0.45)
        self.declare_parameter('bypass_speed',    0.10)
        self.declare_parameter('bypass_cooldown', 2.5)

        self.cruise_speed    = self.get_parameter('cruise_speed').value
        self.center_kp       = self.get_parameter('center_kp').value
        self.corner_dist     = self.get_parameter('corner_distance').value
        self.clear_dist      = self.get_parameter('clear_distance').value
        self.alert_dist      = self.get_parameter('alert_distance').value
        self.alert_angle     = math.radians(self.get_parameter('alert_angle_deg').value)
        self.side_lo         = math.radians(self.get_parameter('side_lo_deg').value)
        self.side_hi         = math.radians(self.get_parameter('side_hi_deg').value)
        self.turn_speed      = self.get_parameter('turn_speed').value
        self.bypass_angle    = math.radians(self.get_parameter('bypass_angle_deg').value)
        self.bypass_forward  = self.get_parameter('bypass_forward').value
        self.bypass_speed    = self.get_parameter('bypass_speed').value
        self.bypass_cooldown = self.get_parameter('bypass_cooldown').value

        self.state         = State.CRUCERO
        self.closest_front = float('inf')
        self.d_wall_pos    = float('inf')  # pared al +theta (~izq)
        self.d_wall_neg    = float('inf')  # pared al -theta (~der)

        self._caja_confirmada = False
        self._caja_ts         = 0.0
        self._last_bypass_ts  = 0.0
        self._t0              = None
        self._bypass_dir      = 1.0
        self._bypass_step     = 0
        self._log_timer       = 0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan',            self._scan_cb, qos)
        self.create_subscription(PoseArray, '/cajas_avistadas', self._cajas_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self._loop)

        self.get_logger().info('BehaviorFSM listo: centrado+esquinas(der)+cajas')

    # ---------------------------------------------------------------- callbacks
    def _cajas_cb(self, msg):
        if msg.poses:
            self._caja_confirmada = True
            self._caja_ts = self._now()
        else:
            self._caja_confirmada = False

    def _scan_cb(self, msg):
        a0 = msg.angle_min
        da = msg.angle_increment

        front = float('inf')
        d_pos = float('inf')
        d_neg = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            theta = a0 + i * da
            ath   = abs(theta)

            # sector frontal: cerca de ±pi
            if abs(ath - math.pi) <= self.alert_angle:
                front = min(front, r)
            # sector lateral para centrado: entre side_lo y side_hi
            elif self.side_lo <= ath <= self.side_hi:
                if theta > 0:
                    d_pos = min(d_pos, r)
                else:
                    d_neg = min(d_neg, r)

        self.closest_front = front
        self.d_wall_pos    = d_pos
        self.d_wall_neg    = d_neg

    # ---------------------------------------------------------------- helpers
    def _pub(self, lin, ang):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _change(self, s):
        self.get_logger().info(f'{self.state.name} -> {s.name}')
        self.state = s
        self._t0   = self._now()

    def _centering(self):
        """omega proporcional para centrarse entre paredes."""
        pos_ok = math.isfinite(self.d_wall_pos)
        neg_ok = math.isfinite(self.d_wall_neg)
        if pos_ok and neg_ok:
            # d_wall_pos = pared izquierda, d_wall_neg = pared derecha
            # si izq < der → robot cerca de izq → girar derecha (omega negativo)
            err   = self.d_wall_pos - self.d_wall_neg
            omega = self.center_kp * err
            return max(-0.35, min(0.35, omega))
        return 0.0

    # ---------------------------------------------------------------- FSM
    def _loop(self):
        now = self._now()
        if now - self._caja_ts > 0.5:
            self._caja_confirmada = False

        in_cd = (now - self._last_bypass_ts) < self.bypass_cooldown
        s     = self.state

        if s == State.CRUCERO:
            omega = self._centering()

            # rampa de frenado suave: 20cm antes de la distancia de giro
            margen = 0.20
            if self.closest_front < self.corner_dist + margen:
                factor = max(0.0, (self.closest_front - self.corner_dist) / margen)
                spd = self.cruise_speed * factor
            else:
                spd = self.cruise_speed

            self._pub(spd, omega)

            self._log_timer += 1
            if self._log_timer >= 20:
                self._log_timer = 0
                self.get_logger().info(
                    f'[CRUCERO] frente={self.closest_front:.2f}m '
                    f'L={self.d_wall_pos:.2f} R={self.d_wall_neg:.2f} '
                    f'omega={omega:+.2f}')

            if not in_cd:
                if self._caja_confirmada and self.closest_front < self.alert_dist:
                    self._change(State.CAJA_DETECTADA)
                elif not self._caja_confirmada and self.closest_front < self.corner_dist:
                    self.get_logger().info(
                        f'[ESQUINA] {self.closest_front:.2f}m — giro derecha')
                    self._change(State.GIRAR)

        elif s == State.GIRAR:
            # gira a la DERECHA hasta que el frente quede libre
            self._pub(0.0, -self.turn_speed)
            if self.closest_front > self.clear_dist:
                self.get_logger().info(
                    f'[ESQUINA OK] frente libre={self.closest_front:.2f}m')
                self._last_bypass_ts = now
                self._change(State.CRUCERO)

        elif s == State.CAJA_DETECTADA:
            self._pub(0.0, 0.0)
            # rodear por el lado con más espacio
            self._bypass_dir = 1.0 if self.d_wall_neg > self.d_wall_pos else -1.0
            lado = 'derecha' if self._bypass_dir > 0 else 'izquierda'
            self.get_logger().info(
                f'Parada | frente={self.closest_front:.2f}m | rodeo por {lado}')
            self._change(State.PARAR)

        elif s == State.PARAR:
            self._pub(0.0, 0.0)
            self._change(State.ESPERAR_3S)

        elif s == State.ESPERAR_3S:
            self._pub(0.0, 0.0)
            if now - self._t0 >= 3.0:
                self._bypass_step = 0
                self._change(State.RODEAR)

        elif s == State.RODEAR:
            self._bypass()

    # ---------------------------------------------------------------- rodeo
    def _bypass(self):
        elapsed      = self._now() - self._t0
        d            = self._bypass_dir
        turn_time    = self.bypass_angle   / self.turn_speed
        forward_time = self.bypass_forward / self.bypass_speed

        if self._bypass_step == 0:
            self._pub(0.0, d * self.turn_speed)
            if elapsed >= turn_time:
                self._bypass_step = 1
                self._t0 = self._now()
                self.get_logger().info('Rodeo: giro OK')
        elif self._bypass_step == 1:
            self._pub(self.bypass_speed, 0.0)
            if elapsed >= forward_time:
                self._bypass_step = 2
                self._t0 = self._now()
                self.get_logger().info('Rodeo: avance OK')
        elif self._bypass_step == 2:
            self._pub(0.0, -d * self.turn_speed)
            if elapsed >= turn_time:
                self._bypass_step = 3
                self._t0 = self._now()
                self.get_logger().info('Rodeo: vuelta OK')
        elif self._bypass_step == 3:
            self._pub(0.0, 0.0)
            self._last_bypass_ts = self._now()
            self._change(State.CRUCERO)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
