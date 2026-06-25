"""
behavior_fsm.py — Parte B del RC3

Solo reacciona cuando box_detector confirma una caja en /cajas_avistadas.
NO para por paredes ni esquinas del jirón.

LiDAR: 0 grados apunta hacia ATRAS. Frente = +/-180 grados.
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
    CAJA_DETECTADA = auto()
    PARAR          = auto()
    ESPERAR_3S     = auto()
    RODEAR         = auto()
    GIRAR          = auto()   # girar en esquina del circuito


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        self.declare_parameter('cruise_speed',     0.28)
        self.declare_parameter('alert_distance',   0.55)
        self.declare_parameter('stop_distance',    0.22)
        self.declare_parameter('alert_angle_deg',  45.0)
        self.declare_parameter('side_angle_deg',   80.0)
        self.declare_parameter('bypass_angle_deg', 18.0)
        self.declare_parameter('bypass_forward',   0.50)
        self.declare_parameter('bypass_speed',     0.10)
        self.declare_parameter('turn_speed',       0.40)
        self.declare_parameter('bypass_cooldown',   2.5)
        self.declare_parameter('corner_distance',   0.40)
        self.declare_parameter('corner_turn_deg',   88.0)

        self.cruise_speed    = self.get_parameter('cruise_speed').value
        self.alert_dist      = self.get_parameter('alert_distance').value
        self.stop_dist       = self.get_parameter('stop_distance').value
        self.alert_angle     = math.radians(self.get_parameter('alert_angle_deg').value)
        self.side_angle      = math.radians(self.get_parameter('side_angle_deg').value)
        self.bypass_angle    = math.radians(self.get_parameter('bypass_angle_deg').value)
        self.bypass_forward  = self.get_parameter('bypass_forward').value
        self.bypass_speed    = self.get_parameter('bypass_speed').value
        self.turn_speed      = self.get_parameter('turn_speed').value
        self.bypass_cooldown  = self.get_parameter('bypass_cooldown').value
        self.corner_dist      = self.get_parameter('corner_distance').value
        self.corner_turn_rad  = math.radians(self.get_parameter('corner_turn_deg').value)

        self.state         = State.CRUCERO
        self.closest_front = float('inf')
        self.dist_right    = float('inf')
        self.dist_left     = float('inf')
        self._bypass_dir   = -1.0
        self._t0           = None
        self._bypass_step  = 0
        self._log_timer    = 0
        self._last_bypass_ts = 0.0   # cooldown: evita retrigger inmediato

        # caja confirmada por box_detector (no para por paredes)
        self._caja_confirmada = False
        self._caja_ts         = 0.0

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan,  '/scan',           self._scan_cb,  qos_sensor)
        self.create_subscription(PoseArray,  '/cajas_avistadas', self._cajas_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self._loop)

        self.get_logger().info('BehaviorFSM listo — CRUCERO + esquinas + cajas')

    # ---------------------------------------------------------------- cajas
    def _cajas_cb(self, msg):
        if len(msg.poses) > 0:
            self._caja_confirmada = True
            self._caja_ts = self._now()
        else:
            self._caja_confirmada = False

    # ---------------------------------------------------------------- scan
    def _scan_cb(self, msg):
        a0   = msg.angle_min
        da   = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max

        front = float('inf')
        right = float('inf')
        left  = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            theta = a0 + i * da
            dist_to_front = abs(abs(theta) - math.pi)

            if dist_to_front <= self.alert_angle:
                front = min(front, r)
            elif self.alert_angle < theta <= self.side_angle:
                right = min(right, r)
            elif -self.side_angle <= theta < -self.alert_angle:
                left = min(left, r)

        self.closest_front = front
        self.dist_right    = right
        self.dist_left     = left

    # ---------------------------------------------------------------- vel
    def _pub(self, lin, ang):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _change(self, new_state):
        self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state = new_state
        self._t0   = self._now()

    # ---------------------------------------------------------------- FSM
    def _loop(self):
        now = self._now()

        # cajas_avistadas expira si no llega nueva en 0.5s
        if now - self._caja_ts > 0.5:
            self._caja_confirmada = False

        s = self.state

        if s == State.CRUCERO:
            # rampa de frenado: si hay pared cerca, reduce velocidad gradualmente
            freno_dist = self.corner_dist + 0.20   # empieza a frenar 20cm antes
            if self.closest_front < freno_dist:
                factor = max(0.3, (self.closest_front - self.corner_dist) / 0.20)
                spd = self.cruise_speed * factor
            else:
                spd = self.cruise_speed
            self._pub(spd, 0.0)

            self._log_timer += 1
            if self._log_timer >= 20:
                self._log_timer = 0
                self.get_logger().info(
                    f'[CRUCERO] frente={self.closest_front:.2f}m spd={spd:.2f} '
                    f'caja={self._caja_confirmada}')

            in_cooldown = (now - self._last_bypass_ts) < self.bypass_cooldown

            if self._caja_confirmada and self.closest_front < self.alert_dist and not in_cooldown:
                self._change(State.CAJA_DETECTADA)
            elif not self._caja_confirmada and self.closest_front < self.corner_dist and not in_cooldown:
                self.get_logger().info(
                    f'[ESQUINA] pared a {self.closest_front:.2f}m — girando izquierda')
                self._change(State.GIRAR)

        elif s == State.CAJA_DETECTADA:
            self._pub(0.0, 0.0)
            self.get_logger().info(
                f'Parada a {self.closest_front:.2f}m | '
                f'der={self.dist_right:.2f}m | izq={self.dist_left:.2f}m')
            self._bypass_dir = 1.0 if self.dist_left > self.dist_right else -1.0
            lado = 'IZQUIERDA' if self._bypass_dir > 0 else 'DERECHA'
            self.get_logger().info(f'Rodeo por {lado}')
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

        elif s == State.GIRAR:
            # gira izquierda (sentido antihorario = positivo en ROS)
            turn_time = self.corner_turn_rad / self.turn_speed
            self._pub(0.0, self.turn_speed)
            if now - self._t0 >= turn_time:
                self._last_bypass_ts = now   # cooldown breve post-esquina
                self.get_logger().info('Esquina completada — CRUCERO')
                self._change(State.CRUCERO)

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
                self.get_logger().info('Rodeo: giro lateral OK')

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
                self.get_logger().info('Rodeo: vuelta al carril OK')

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
