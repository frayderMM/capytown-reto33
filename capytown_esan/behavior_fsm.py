"""
behavior_fsm.py — Parte B del RC3
Máquina de estados reactiva para detectar y rodear cajas.

Estados:
  CRUCERO → CAJA_DETECTADA → PARAR → ESPERAR_3S → RODEAR → CRUCERO
"""
import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class State(Enum):
    CRUCERO        = auto()
    CAJA_DETECTADA = auto()
    PARAR          = auto()
    ESPERAR_3S     = auto()
    RODEAR         = auto()


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # --- parámetros ---
        self.declare_parameter('cruise_speed',     0.15)   # velocidad crucero [m/s]
        self.declare_parameter('alert_distance',   0.30)   # distancia de alerta [m]
        self.declare_parameter('stop_distance',    0.17)   # distancia mínima de parada [m]
        self.declare_parameter('alert_angle_deg',  45.0)   # semisector frontal [°]
        self.declare_parameter('bypass_angle_deg', 30.0)   # ángulo de desvío [°]
        self.declare_parameter('bypass_forward',   0.40)   # avance durante rodeo [m]
        self.declare_parameter('bypass_speed',     0.10)   # velocidad durante rodeo [m/s]
        self.declare_parameter('turn_speed',       0.40)   # velocidad angular [rad/s]

        self.cruise_speed   = self.get_parameter('cruise_speed').value
        self.alert_dist     = self.get_parameter('alert_distance').value
        self.stop_dist      = self.get_parameter('stop_distance').value
        self.alert_angle    = math.radians(self.get_parameter('alert_angle_deg').value)
        self.bypass_angle   = math.radians(self.get_parameter('bypass_angle_deg').value)
        self.bypass_forward = self.get_parameter('bypass_forward').value
        self.bypass_speed   = self.get_parameter('bypass_speed').value
        self.turn_speed     = self.get_parameter('turn_speed').value

        self.state          = State.CRUCERO
        self.closest_front  = float('inf')
        self._t0            = None   # timestamp de inicio de un estado temporal
        self._bypass_step   = 0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # loop a 20 Hz
        self.create_timer(0.05, self._loop)
        self.get_logger().info('BehaviorFSM listo — estado inicial: CRUCERO')

    # ------------------------------------------------------------------ scan
    def _scan_cb(self, msg):
        a0   = msg.angle_min
        da   = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max

        closest = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            theta = a0 + i * da
            if abs(theta) <= self.alert_angle:
                closest = min(closest, r)
        self.closest_front = closest

    # ------------------------------------------------------------------ vel
    def _pub(self, lin, ang):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _change(self, new_state):
        self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.state = new_state
        self._t0   = self._now()

    # ------------------------------------------------------------------ FSM
    def _loop(self):
        s = self.state

        if s == State.CRUCERO:
            self._pub(self.cruise_speed, 0.0)
            if self.closest_front < self.alert_dist:
                self._change(State.CAJA_DETECTADA)

        elif s == State.CAJA_DETECTADA:
            # frena suavemente hasta llegar a stop_dist
            if self.closest_front > self.stop_dist:
                self._pub(0.05, 0.0)
            else:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f'Parada a {self.closest_front:.2f} m de la caja')
                self._change(State.PARAR)

        elif s == State.PARAR:
            self._pub(0.0, 0.0)
            self._change(State.ESPERAR_3S)

        elif s == State.ESPERAR_3S:
            self._pub(0.0, 0.0)
            if self._now() - self._t0 >= 3.0:
                self._bypass_step = 0
                self._change(State.RODEAR)

        elif s == State.RODEAR:
            self._bypass()

    # ------------------------------------------------------------------ rodeo
    def _bypass(self):
        elapsed = self._now() - self._t0
        turn_time    = self.bypass_angle   / self.turn_speed    # s para girar ~30°
        forward_time = self.bypass_forward / self.bypass_speed  # s para avanzar ~40 cm

        if self._bypass_step == 0:
            # paso 1: girar a la derecha
            self._pub(0.0, -self.turn_speed)
            if elapsed >= turn_time:
                self._bypass_step = 1
                self._t0 = self._now()
                self.get_logger().info('Rodeo: giro derecha OK')

        elif self._bypass_step == 1:
            # paso 2: avanzar sobrepasando la caja
            self._pub(self.bypass_speed, 0.0)
            if elapsed >= forward_time:
                self._bypass_step = 2
                self._t0 = self._now()
                self.get_logger().info('Rodeo: avance OK')

        elif self._bypass_step == 2:
            # paso 3: girar a la izquierda para volver al carril
            self._pub(0.0, self.turn_speed)
            if elapsed >= turn_time:
                self._bypass_step = 3
                self._t0 = self._now()
                self.get_logger().info('Rodeo: giro izquierda OK')

        elif self._bypass_step == 3:
            self._pub(0.0, 0.0)
            self._change(State.CRUCERO)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
