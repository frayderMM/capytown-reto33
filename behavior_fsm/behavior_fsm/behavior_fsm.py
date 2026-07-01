#!/usr/bin/env python3
"""
behavior_fsm.py
---------------
PARTE B del reto: "El Guardian".

Maquina de estados (FSM) que vigila el sector frontal con el LiDAR y reacciona
ante las cajas:

    CRUCERO          -> avanza recto vigilando el frente (+/- 45 grados)
    CAJA_DETECTADA   -> hay obstaculo cerca: reduce velocidad
    PARAR            -> obstaculo muy cerca: detiene el robot
    ESPERAR_3S       -> espera 3 segundos detenido frente a la caja
    RODEAR           -> maniobra de evasion: gira +30 / avanza / gira -30

Distancias (sector frontal):
    alerta  < 0.30 m  -> CAJA_DETECTADA
    parada  < 0.15 m  -> PARAR -> ESPERAR_3S -> RODEAR

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
CAJA_DETECTADA = 'CAJA_DETECTADA'
PARAR = 'PARAR'
ESPERAR_3S = 'ESPERAR_3S'
RODEAR = 'RODEAR'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ---- Parametros ----
        self.declare_parameter('sector_frontal_deg', 45.0)  # +/- grados al frente
        self.declare_parameter('dist_alerta', 0.30)         # m -> CAJA_DETECTADA
        self.declare_parameter('dist_parada', 0.15)         # m -> PARAR
        self.declare_parameter('vel_crucero', 0.15)         # m/s
        self.declare_parameter('vel_precaucion', 0.07)      # m/s
        self.declare_parameter('vel_giro', 0.5)             # rad/s
        self.declare_parameter('espera_seg', 3.0)           # s detenido
        self.declare_parameter('angulo_rodeo_deg', 30.0)    # grados de giro del rodeo
        self.declare_parameter('avance_rodeo_seg', 2.0)     # s de avance al rodear

        self.sector = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.d_alerta = self.get_parameter('dist_alerta').value
        self.d_parada = self.get_parameter('dist_parada').value
        self.v_crucero = self.get_parameter('vel_crucero').value
        self.v_precaucion = self.get_parameter('vel_precaucion').value
        self.w_giro = self.get_parameter('vel_giro').value
        self.espera = self.get_parameter('espera_seg').value
        self.ang_rodeo = math.radians(self.get_parameter('angulo_rodeo_deg').value)
        self.t_avance = self.get_parameter('avance_rodeo_seg').value

        # ---- Estado de la FSM ----
        self.estado = CRUCERO
        self.dist_frente = float('inf')
        self.t_inicio_estado = self.get_clock().now()
        # Subfases del rodeo: 0=girar+, 1=avanzar, 2=girar-, 3=fin
        self.fase_rodeo = 0
        # Corrección lateral del wall_follower (rad/s). Solo se aplica en CRUCERO.
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
        """Recibe la corrección angular del wall_follower (rad/s)."""
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
    def _tiempo_en_estado(self) -> float:
        return (self.get_clock().now() - self.t_inicio_estado).nanoseconds * 1e-9

    def _cambiar(self, nuevo):
        if nuevo != self.estado:
            self.get_logger().info(f'{self.estado} -> {nuevo} '
                                   f'(d_frente={self.dist_frente:.2f} m)')
            self.estado = nuevo
            self.t_inicio_estado = self.get_clock().now()

    def _publicar(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    def loop_control(self):
        d = self.dist_frente

        if self.estado == CRUCERO:
            if d < self.d_parada:
                self._cambiar(PARAR)
            elif d < self.d_alerta:
                self._cambiar(CAJA_DETECTADA)
            else:
                self._publicar(self.v_crucero, self._w_lateral)

        elif self.estado == CAJA_DETECTADA:
            if d < self.d_parada:
                self._cambiar(PARAR)
            elif d >= self.d_alerta:
                self._cambiar(CRUCERO)  # se despejo
            else:
                self._publicar(self.v_precaucion, 0.0)  # avanza con cautela

        elif self.estado == PARAR:
            self._publicar(0.0, 0.0)
            self._cambiar(ESPERAR_3S)

        elif self.estado == ESPERAR_3S:
            self._publicar(0.0, 0.0)
            if self._tiempo_en_estado() >= self.espera:
                self.fase_rodeo = 0
                self._cambiar(RODEAR)

        elif self.estado == RODEAR:
            self._ejecutar_rodeo()

    # ------------------------------------------------------------------
    def _ejecutar_rodeo(self):
        """Maniobra de evasion en 3 fases: girar +ang, avanzar, girar -ang."""
        # Tiempo necesario para girar 'ang_rodeo' a 'w_giro' rad/s.
        t_giro = self.ang_rodeo / self.w_giro
        t = self._tiempo_en_estado()

        if self.fase_rodeo == 0:          # girar a la izquierda (+)
            if t < t_giro:
                self._publicar(0.0, self.w_giro)
            else:
                self.fase_rodeo = 1
                self.t_inicio_estado = self.get_clock().now()

        elif self.fase_rodeo == 1:        # avanzar para bordear la caja
            if t < self.t_avance:
                self._publicar(self.v_crucero, 0.0)
            else:
                self.fase_rodeo = 2
                self.t_inicio_estado = self.get_clock().now()

        elif self.fase_rodeo == 2:        # girar a la derecha (-) para reorientar
            if t < t_giro:
                self._publicar(0.0, -self.w_giro)
            else:
                self.fase_rodeo = 3
                self.t_inicio_estado = self.get_clock().now()

        else:                              # fin del rodeo -> volver a crucero
            self._publicar(0.0, 0.0)
            self._cambiar(CRUCERO)


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
