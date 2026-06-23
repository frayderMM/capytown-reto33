"""
behavior_fsm.py — Parte B del RC3
Máquina de estados reactiva para detectar y rodear cajas.

Pista: jirón de 60 cm, cajas de 20×20 cm pegadas a las paredes.
El bypass elige automáticamente el lado más libre mirando el scan lateral.

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
        self.declare_parameter('cruise_speed',     0.15)
        self.declare_parameter('alert_distance',   0.35)
        self.declare_parameter('stop_distance',    0.17)
        self.declare_parameter('alert_angle_deg',  40.0)
        self.declare_parameter('side_angle_deg',   90.0)   # sector lateral para elegir lado
        self.declare_parameter('bypass_angle_deg', 32.0)
        self.declare_parameter('bypass_forward',   0.55)
        self.declare_parameter('bypass_speed',     0.10)
        self.declare_parameter('turn_speed',       0.40)

        self.cruise_speed   = self.get_parameter('cruise_speed').value
        self.alert_dist     = self.get_parameter('alert_distance').value
        self.stop_dist      = self.get_parameter('stop_distance').value
        self.alert_angle    = math.radians(self.get_parameter('alert_angle_deg').value)
        self.side_angle     = math.radians(self.get_parameter('side_angle_deg').value)
        self.bypass_angle   = math.radians(self.get_parameter('bypass_angle_deg').value)
        self.bypass_forward = self.get_parameter('bypass_forward').value
        self.bypass_speed   = self.get_parameter('bypass_speed').value
        self.turn_speed     = self.get_parameter('turn_speed').value

        self.state         = State.CRUCERO
        self.closest_front = float('inf')
        self.dist_left     = float('inf')   # distancia libre al costado izquierdo
        self.dist_right    = float('inf')   # distancia libre al costado derecho
        self._bypass_dir   = -1.0           # +1 = izquierda, -1 = derecha
        self._t0           = None
        self._bypass_step  = 0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self._loop)   # 20 Hz
        self.get_logger().info('BehaviorFSM listo — CRUCERO')

    # ------------------------------------------------------------------ scan
    def _scan_cb(self, msg):
        a0   = msg.angle_min
        da   = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max

        front = float('inf')
        left  = float('inf')
        right = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            theta = a0 + i * da

            # sector frontal
            if abs(theta) <= self.alert_angle:
                front = min(front, r)

            # sector lateral izquierdo (θ positivo en ROS = izquierda)
            elif self.alert_angle < theta <= self.side_angle:
                left = min(left, r)

            # sector lateral derecho (θ negativo = derecha)
            elif -self.side_angle <= theta < -self.alert_angle:
                right = min(right, r)

        self.closest_front = front
        self.dist_left     = left
        self.dist_right    = right

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
            # frena hasta llegar a stop_dist
            if self.closest_front > self.stop_dist:
                self._pub(0.05, 0.0)
            else:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f'Parada a {self.closest_front:.2f} m | '
                    f'izq={self.dist_left:.2f} m | der={self.dist_right:.2f} m')
                # elegir lado más libre para el rodeo
                self._bypass_dir = 1.0 if self.dist_left > self.dist_right else -1.0
                lado = 'IZQUIERDA' if self._bypass_dir > 0 else 'DERECHA'
                self.get_logger().info(f'Rodeo por {lado}')
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
        elapsed      = self._now() - self._t0
        d            = self._bypass_dir               # +1 izq / -1 der
        turn_time    = self.bypass_angle / self.turn_speed
        forward_time = self.bypass_forward / self.bypass_speed

        if self._bypass_step == 0:
            # desviar hacia el lado libre
            self._pub(0.0, d * self.turn_speed)
            if elapsed >= turn_time:
                self._bypass_step = 1
                self._t0 = self._now()
                self.get_logger().info('Rodeo: giro lateral OK')

        elif self._bypass_step == 1:
            # avanzar hasta sobrepasar la caja
            self._pub(self.bypass_speed, 0.0)
            if elapsed >= forward_time:
                self._bypass_step = 2
                self._t0 = self._now()
                self.get_logger().info('Rodeo: avance OK')

        elif self._bypass_step == 2:
            # volver al carril
            self._pub(0.0, -d * self.turn_speed)
            if elapsed >= turn_time:
                self._bypass_step = 3
                self._t0 = self._now()
                self.get_logger().info('Rodeo: vuelta al carril OK')

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
