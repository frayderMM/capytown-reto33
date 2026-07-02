#!/usr/bin/env python3
"""
behavior_fsm.py
---------------
PARTE B del reto: "El Guardian".

FSM de 2 estados que vigila el sector frontal con el LiDAR:

    CRUCERO  -> avanza recto, autoalineandose con la pared derecha
                (correccion que llega de wall_follower por /lateral_correction)
    EVASION  -> hay una caja cerca al frente: gira hacia la IZQUIERDA de forma
                PROPORCIONAL a que tan cerca esta (mas cerca => gira mas fuerte),
                sin parar, sin esperar tiempos fijos y sin angulo de giro fijo.

Ya no hay PARAR / ESPERAR_3S / RODEAR con duracion o angulo fijos: la
evasion es continua y reactiva a la distancia real medida en cada vuelta
de /scan, y termina en cuanto el sector frontal vuelve a estar libre.

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# Estados de la FSM.
CRUCERO = 'CRUCERO'
EVASION = 'EVASION'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ---- Parametros ----
        self.declare_parameter('sector_frontal_deg', 45.0)  # +/- grados al frente
        self.declare_parameter('dist_alerta', 0.30)         # m -> entra a EVASION
        self.declare_parameter('dist_parada', 0.15)         # m -> frena el avance (sigue girando)
        self.declare_parameter('vel_crucero', 0.15)         # m/s
        self.declare_parameter('vel_evasion', 0.08)         # m/s mientras evade
        self.declare_parameter('w_evasion_max', 0.6)        # rad/s giro maximo hacia la izquierda

        self.sector = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.d_alerta = self.get_parameter('dist_alerta').value
        self.d_parada = self.get_parameter('dist_parada').value
        self.v_crucero = self.get_parameter('vel_crucero').value
        self.v_evasion = self.get_parameter('vel_evasion').value
        self.w_evasion_max = self.get_parameter('w_evasion_max').value

        # ---- Estado de la FSM ----
        self.estado = CRUCERO
        self.dist_frente = float('inf')
        # Correccion lateral del wall_follower (rad/s). Solo se aplica en CRUCERO.
        self._w_lateral = 0.0

        # ---- ROS I/O ----
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.create_subscription(Float32, '/lateral_correction',
                                 self._cb_lateral, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        # Bucle de control a 10 Hz: la FSM decide aqui, no en el callback del scan.
        self.create_timer(0.1, self.loop_control)

        self.get_logger().info('behavior_fsm iniciado en estado CRUCERO.')

    # ------------------------------------------------------------------
    def _cb_lateral(self, msg: Float32):
        """Recibe la correccion angular del wall_follower (rad/s)."""
        self._w_lateral = msg.data

    # ------------------------------------------------------------------
    def cb_scan(self, msg: LaserScan):
        """Calcula la distancia minima dentro del sector frontal +/- self.sector."""
        d_min = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            # Normalizar a [-pi, pi] para comparar con el sector frontal (centrado en 0).
            ang = math.atan2(math.sin(ang), math.cos(ang))
            if abs(ang) <= self.sector:
                d_min = min(d_min, r)
        self.dist_frente = d_min

    # ------------------------------------------------------------------
    def _cambiar(self, nuevo):
        if nuevo != self.estado:
            self.get_logger().info(f'{self.estado} -> {nuevo} '
                                   f'(d_frente={self.dist_frente:.2f} m)')
            self.estado = nuevo

    def _publicar(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    def loop_control(self):
        d = self.dist_frente

        if self.estado == CRUCERO:
            if d < self.d_alerta:
                self._cambiar(EVASION)
            else:
                self._publicar(self.v_crucero, self._w_lateral)

        elif self.estado == EVASION:
            if d >= self.d_alerta:
                self._cambiar(CRUCERO)
                return
            # Cuanto mas cerca esta la caja, mas fuerte gira hacia la izquierda.
            factor = max(0.0, min(1.0, (self.d_alerta - d) / self.d_alerta))
            w = self.w_evasion_max * factor          # w > 0 -> gira IZQUIERDA
            v = 0.0 if d < self.d_parada else self.v_evasion
            self._publicar(v, w)


def main(args=None):
    rclpy.init(args=args)
    nodo = BehaviorFSM()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo._publicar(0.0, 0.0)  # asegurar que el robot quede detenido
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
