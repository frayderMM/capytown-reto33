#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Control reactivo continuo (sin estados discretos, sin seguir ninguna
pared ni mantener distancia a ningun lado): en cada ciclo calcula el
espacio libre real hasta el borde del robot (la distancia que da el
LiDAR menos el offset fisico LiDAR→borde, distinto al frente/atras/
costados) y combina dos señales graduales:

  - velocidad: progresiva, baja segun se cierra el espacio al frente.
  - giro: progresivo, solo reacciona cuando hay algo perpendicular al
    frente (no a paredes laterales paralelas al avance), eligiendo el
    lado con MAS espacio libre para girar.

La parada de emergencia (omnidireccional) es la unica que vigila los
costados, usando los offsets reales LiDAR->borde para no chocar por
ningun lado.

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  30.0)  # +/- grados del cono frontal
        self.declare_parameter('sector_lateral_lo',   60.0)
        self.declare_parameter('sector_lateral_hi',  120.0)
        self.declare_parameter('excluir_atras_deg',   60.0)  # +/- grados detras del robot
                                                               # excluidos del minimo global:
                                                               # el cable/soporte del LiDAR
                                                               # queda justo ahi y se leia como
                                                               # obstaculo fijo a ~12cm, disparando
                                                               # EMERGENCIA en casi todos los ciclos

        # --- Offsets LiDAR -> borde fisico del robot (NO es el mismo en
        # cada direccion) -- las distancias del LiDAR se corrigen con esto
        # antes de comparar contra cualquier umbral, para que el margen
        # real sea hasta el borde del robot, no hasta el sensor.
        self.declare_parameter('offset_frente', 0.15)  # m  LiDAR -> borde frontal
        self.declare_parameter('offset_atras',  0.10)  # m  LiDAR -> borde trasero
        self.declare_parameter('offset_lados',  0.08)  # m  LiDAR -> borde lateral

        # --- Distancias de reaccion (ya en espacio libre real, post-offset) ---
        self.declare_parameter('dist_alerta',     0.38)  # m  empieza a frenar y a anticipar el giro
        self.declare_parameter('dist_obstaculo',  0.30)  # m  giro a maxima intensidad
        self.declare_parameter('dist_emergencia', 0.05)  # m  margen real minimo antes del stop total

        # --- Velocidades ---
        self.declare_parameter('vel_crucero', 0.22)
        self.declare_parameter('vel_min',     0.08)
        self.declare_parameter('w_giro_max',  0.45)  # rad/s  maximo giro al evadir un frente bloqueado

        self.front_rad = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector    = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo    = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi    = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.atras_lim = math.pi - math.radians(self.get_parameter('excluir_atras_deg').value) / 2.0

        self.off_frente = self.get_parameter('offset_frente').value
        self.off_atras  = self.get_parameter('offset_atras').value
        self.off_lados  = self.get_parameter('offset_lados').value

        self.d_alerta = self.get_parameter('dist_alerta').value
        self.d_obst   = self.get_parameter('dist_obstaculo').value
        self.d_emerg  = self.get_parameter('dist_emergencia').value

        self.v_cruise  = self.get_parameter('vel_crucero').value
        self.v_min     = self.get_parameter('vel_min').value
        self.w_giro_max = self.get_parameter('w_giro_max').value

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente = float('inf')  # cono ancho frontal
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self.dist_min    = float('inf')  # minimo global, todos los angulos

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.cb_scan, _qos)
        self.pub_cmd    = self.create_publisher(Twist,   '/cmd_vel',     10)
        self.pub_estado = self.create_publisher(String,  '/fsm_state',   10)
        self.pub_parada = self.create_publisher(Float32, '/parada_dist', 10)
        self.create_timer(0.1, self.loop_control)

        self._en_evasion = False  # para publicar /parada_dist solo al entrar a la zona de evasion

        self.get_logger().info('BehaviorFSM listo — evasion omnidireccional continua')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def cb_scan(self, msg: LaserScan):
        d_f = d_l = d_r = d_min = float('inf')

        for i, r in enumerate(msg.ranges):
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            abs_af = abs(af)
            valid  = math.isfinite(r) and msg.range_min <= r <= msg.range_max
            if not valid:
                continue

            # Minimo global (casi todos los angulos, excepto el cono
            # trasero donde esta el cable/soporte del LiDAR): cubre la
            # zona "ciega" entre el cono frontal y el sector lateral, que
            # de otra forma no se mide en ningun lado. Solo para emergencia.
            if abs_af <= self.atras_lim:
                d_min = min(d_min, r)

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
        self.dist_min    = d_min

    # ── Helpers ───────────────────────────────────────────────────────────
    def _pub(self, v: float, w: float, estado: str):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String(); s.data = estado
        self.pub_estado.publish(s)

    def _vel_adaptativa(self, c_frente: float) -> float:
        if c_frente >= self.d_alerta:
            return self.v_cruise
        ratio = (c_frente - self.d_obst) / (self.d_alerta - self.d_obst)
        return self.v_min + max(0.0, min(1.0, ratio)) * (self.v_cruise - self.v_min)

    # ── Control principal ────────────────────────────────────────────────
    def loop_control(self):
        # Espacio libre real hasta el borde del robot (LiDAR - offset),
        # no la distancia cruda del sensor. El offset lateral (el mas
        # chico) se usa para el minimo global por ser el mas conservador,
        # ya que no se sabe de que direccion vino ese minimo.
        c_frente = self.dist_frente - self.off_frente
        c_izq    = (self.dist_izq - self.off_lados) if math.isfinite(self.dist_izq) else float('inf')
        c_der    = (self.dist_der - self.off_lados) if math.isfinite(self.dist_der) else float('inf')
        c_min    = self.dist_min - self.off_lados

        # Avanzar depende SOLO del frente -- los costados nunca lo frenan.
        v = self._vel_adaptativa(c_frente)

        # Emergencia de frente: ahi si no hay forma de "girar para evitarlo"
        # sin parar primero (seguir de frente lo choca de lleno).
        if c_frente < self.d_emerg:
            self.get_logger().warn(
                f'EMERGENCIA frente={c_frente:.2f}m — stop total', throttle_duration_sec=1.0)
            self._pub(0.0, 0.0, 'EMERGENCIA')
            self._en_evasion = False
            return

        # Algo muy cerca por un costado/diagonal (frente libre): en vez de
        # parar, gira al maximo hacia el lado con MAS espacio mientras
        # sigue avanzando -- se recalibra sin detenerse.
        if c_min < self.d_emerg:
            lado = 1.0 if c_izq >= c_der else -1.0
            self.get_logger().warn(
                f'cerca por un costado (margen={c_min:.2f}m) — girando hacia el lado mas libre',
                throttle_duration_sec=1.0)
            self._pub(v, lado * self.w_giro_max, 'RECALIBRAR')
            return

        # Giro: solo reacciona a lo que tiene perpendicular al frente
        # (no a paredes laterales paralelas al avance). Progresivo desde
        # dist_alerta hasta el maximo en dist_obstaculo, eligiendo el
        # lado con MAS espacio libre para girar -- no siempre el mismo.
        w = 0.0
        if c_frente < self.d_alerta:
            ratio = max(0.0, min(1.0, (self.d_alerta - c_frente) / (self.d_alerta - self.d_obst)))
            lado = 1.0 if c_izq >= c_der else -1.0  # +1 = izquierda, -1 = derecha
            w = lado * ratio * self.w_giro_max

            if not self._en_evasion:
                self._en_evasion = True
                d_msg = Float32(); d_msg.data = float(c_frente)
                self.pub_parada.publish(d_msg)
        else:
            self._en_evasion = False

        self.get_logger().info(
            f'c_frente={c_frente:.2f} c_izq={c_izq:.2f} c_der={c_der:.2f}  v={v:.2f} w={w:.2f}',
            throttle_duration_sec=0.5)
        self._pub(v, w, 'EVADIR' if self._en_evasion else 'AVANCE')


def main(args=None):
    rclpy.init(args=args)
    nodo = BehaviorFSM()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo._pub(0.0, 0.0, 'STOP')
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
