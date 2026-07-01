#!/usr/bin/env python3
"""
wall_follower.py — Nodo OPCIONAL de depuración del seguimiento de pared derecha.

El guardián (behavior_fsm.py) ya integra el seguimiento de pared — este nodo
NO es necesario para la corrida y el launch NO lo lanza. Se conserva para
depurar la referencia de pared por separado: publica en /lateral_correction
(Float32) la corrección que calcularía el PD, usando exactamente la misma
percepción (behavior_fsm/percepcion.py), ya con el marco del LiDAR corregido
(lidar_front_deg) — la versión anterior procesaba el scan crudo sin rotar,
por lo que confundía izquierda con derecha en el robot real.

    ros2 run behavior_fsm wall_follower --ros-args --params-file params.yaml
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from behavior_fsm import percepcion as pc


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.declare_parameter('lidar_front_deg', 180.0)
        self.declare_parameter('excluir_atras_deg', 60.0)
        self.declare_parameter('rango_max', 3.5)
        self.declare_parameter('salto_dist', 0.12)
        self.declare_parameter('salto_idx', 5)
        self.declare_parameter('umbral_split', 0.04)
        self.declare_parameter('min_puntos', 4)
        self.declare_parameter('lado_caja_max', 0.32)
        self.declare_parameter('min_long_pared', 0.45)
        self.declare_parameter('cos_lateral_min', 0.55)
        self.declare_parameter('dist_pared', 0.15)     # holgura lado robot→pared
        self.declare_parameter('offset_lados', 0.08)
        self.declare_parameter('Kp', 1.4)
        self.declare_parameter('Kd', 0.20)
        self.declare_parameter('Ka', 1.0)
        self.declare_parameter('max_w', 0.8)
        self.declare_parameter('max_delta_w', 0.15)

        g = lambda n: self.get_parameter(n).value
        self.front_rad = math.radians(g('lidar_front_deg'))
        self.atras_rad = math.radians(g('excluir_atras_deg')) / 2.0
        self.rango_max = g('rango_max')
        self.salto_dist, self.salto_idx = g('salto_dist'), int(g('salto_idx'))
        self.umbral_split, self.min_puntos = g('umbral_split'), int(g('min_puntos'))
        self.lado_caja, self.min_pared = g('lado_caja_max'), g('min_long_pared')
        self.cos_lat = g('cos_lateral_min')
        self.d_obj = g('dist_pared') + g('offset_lados')
        self.Kp, self.Kd, self.Ka = g('Kp'), g('Kd'), g('Ka')
        self.max_w, self.max_dw = g('max_w'), g('max_delta_w')

        self._err_prev, self._w_prev = 0.0, 0.0
        self._t_prev = self.get_clock().now()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._cb, qos)
        self._pub = self.create_publisher(Float32, '/lateral_correction', 10)
        self.get_logger().info(
            f'wall_follower (debug) listo | objetivo LiDAR→pared {self.d_obj:.2f} m')

    def _cb(self, msg: LaserScan):
        pts = pc.filtrar_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max,
                              self.front_rad, self.rango_max, self.atras_rad)
        cls = pc.analizar_scan(pts, self.salto_dist, self.salto_idx,
                               self.umbral_split, self.min_puntos,
                               self.lado_caja, self.min_pared)
        pared = pc.pared_derecha(cls, self.min_pared, self.cos_lat)

        if pared is None:
            w = 0.0
            self._err_prev = 0.0
        else:
            err = pared['d'] - self.d_obj
            now = self.get_clock().now()
            dt = max((now - self._t_prev).nanoseconds * 1e-9, 0.01)
            derr = (err - self._err_prev) / dt
            self._err_prev, self._t_prev = err, now
            w = -self.Kp * err - self.Kd * derr + self.Ka * pared['alpha']
            w = max(-self.max_w, min(self.max_w, w))
        w = max(self._w_prev - self.max_dw, min(self._w_prev + self.max_dw, w))
        self._w_prev = w

        out = Float32()
        out.data = float(w)
        self._pub.publish(out)
        if pared is not None:
            self.get_logger().info(
                f'pared der d={pared["d"]:.2f} m alpha='
                f'{math.degrees(pared["alpha"]):.0f}°  w={w:.2f}',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    nodo = WallFollower()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
