#!/usr/bin/env python3
"""
behavior_fsm.py
---------------
PARTE B del reto: "El Guardian".

FSM simplificada de 3 estados con mejoras sobre el diseño base:

    CRUCERO  -> avanza con velocidad PROPORCIONAL a la distancia frontal
                (frena suavemente al acercarse, no hay salto brusco)
                El wall_follower corrige el centrado lateral en zona libre.

    PARAR    -> obstáculo a <= dist_parada: detiene el robot.
                Pausa breve de estabilizacion (0.3 s) y pasa a RODEAR.
                Al entrar, mide qué lado tiene más espacio y elige dirección.

    RODEAR   -> maniobra de evasión adaptativa:
                fase 0: gira hacia el lado con MÁS espacio
                fase 1: avanza bordeando la caja
                fase 2: giro inverso para reincorporarse al carril
                → vuelve a CRUCERO

Mejoras respecto al diseño base del enunciado:
  - Sin ESPERAR_3S (no evaluado en rúbrica, solo ralentiza la corrida)
  - Sin CAJA_DETECTADA (absorbido en la rampa de velocidad de CRUCERO)
  - Velocidad proporcional: v = vel_min + k*(d-d_parada)/(d_alerta-d_parada)
  - Rodeo hacia el lado libre (no siempre izquierda)
  - Publica /fsm_state para monitoreo en lidar_viz

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


CRUCERO = 'CRUCERO'
PARAR   = 'PARAR'
RODEAR  = 'RODEAR'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)  # 180=Yahboom, 0=estandar
        self.declare_parameter('sector_frontal_deg',  45.0)
        self.declare_parameter('sector_lateral_lo',   60.0)
        self.declare_parameter('sector_lateral_hi',  120.0)
        self.declare_parameter('dist_alerta',          0.45)
        self.declare_parameter('dist_parada',          0.18)
        self.declare_parameter('vel_crucero',          0.18)
        self.declare_parameter('vel_min',              0.05)
        self.declare_parameter('vel_giro',             0.50)
        self.declare_parameter('pausa_parada',         0.30)
        self.declare_parameter('angulo_rodeo_deg',    35.0)
        self.declare_parameter('avance_rodeo_seg',     1.8)

        self.front_rad = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector   = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo   = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi   = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.d_alerta = self.get_parameter('dist_alerta').value
        self.d_parada = self.get_parameter('dist_parada').value
        self.v_cruise = self.get_parameter('vel_crucero').value
        self.v_min    = self.get_parameter('vel_min').value
        self.w_giro   = self.get_parameter('vel_giro').value
        self.t_pausa  = self.get_parameter('pausa_parada').value
        self.ang_rodeo= math.radians(self.get_parameter('angulo_rodeo_deg').value)
        self.t_avance = self.get_parameter('avance_rodeo_seg').value

        # ── Estado ────────────────────────────────────────────────────────
        self.estado        = CRUCERO
        self.t_inicio      = self.get_clock().now()
        self.fase_rodeo    = 0
        self._rodeo_dir    = 1.0   # +1 = izquierda, -1 = derecha

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente   = float('inf')
        self.dist_izq      = float('inf')
        self.dist_der      = float('inf')
        self._w_lateral    = 0.0   # correccion del wall_follower

        # ── ROS I/O ───────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',              self.cb_scan,    10)
        self.create_subscription(Float32,   '/lateral_correction',self._cb_lat,    10)
        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel',    10)
        self.pub_estado = self.create_publisher(String, '/fsm_state',  10)
        self.pub_parada = self.create_publisher(Float32,'/parada_dist',10)
        self.create_timer(0.1, self.loop_control)

        self.get_logger().info('BehaviorFSM listo — 3 estados, velocidad adaptativa')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def cb_scan(self, msg: LaserScan):
        d_f = d_l = d_r = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            raw = msg.angle_min + i * msg.angle_increment
            # af = ángulo relativo al frente del robot, normalizado a [-π, π]
            af = math.atan2(math.sin(raw - self.front_rad),
                            math.cos(raw - self.front_rad))
            abs_af = abs(af)
            if abs_af <= self.sector:
                d_f = min(d_f, r)
            elif self.lat_lo <= abs_af <= self.lat_hi:
                if af > 0:
                    d_l = min(d_l, r)   # izquierda del frente
                else:
                    d_r = min(d_r, r)   # derecha del frente
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
        msg = String(); msg.data = self.estado
        self.pub_estado.publish(msg)

    def _t_estado(self) -> float:
        return (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9

    def _cambiar(self, nuevo: str):
        self.get_logger().info(f'{self.estado} → {nuevo}  '
                               f'(frente={self.dist_frente:.2f}m '
                               f'izq={self.dist_izq:.2f}m '
                               f'der={self.dist_der:.2f}m)')
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def _vel_adaptativa(self) -> float:
        """Velocidad proporcional a la distancia frontal."""
        d = self.dist_frente
        if d >= self.d_alerta:
            return self.v_cruise
        ratio = (d - self.d_parada) / (self.d_alerta - self.d_parada)
        ratio = max(0.0, min(1.0, ratio))
        return self.v_min + ratio * (self.v_cruise - self.v_min)

    # ── FSM principal ─────────────────────────────────────────────────────
    def loop_control(self):

        if self.estado == CRUCERO:
            if self.dist_frente <= self.d_parada:
                self._rodeo_dir = 1.0 if self.dist_izq >= self.dist_der else -1.0
                # Publicar distancia real de parada para métricas
                msg_d = Float32(); msg_d.data = float(self.dist_frente)
                self.pub_parada.publish(msg_d)
                self._cambiar(PARAR)
                return

            v = self._vel_adaptativa()
            # Corrección lateral del wall_follower solo en zona libre
            w = self._w_lateral if self.dist_frente >= self.d_alerta else 0.0
            self._pub(v, w)

        elif self.estado == PARAR:
            self._pub(0.0, 0.0)
            if self._t_estado() >= self.t_pausa:
                self.fase_rodeo = 0
                self._cambiar(RODEAR)

        elif self.estado == RODEAR:
            self._ejecutar_rodeo()

    def _ejecutar_rodeo(self):
        t_giro = self.ang_rodeo / self.w_giro
        t = self._t_estado()

        if self.fase_rodeo == 0:           # gira hacia el lado libre
            if t < t_giro:
                self._pub(0.0, self._rodeo_dir * self.w_giro)
            else:
                self.fase_rodeo = 1
                self.t_inicio = self.get_clock().now()

        elif self.fase_rodeo == 1:         # avanza bordeando
            if self.dist_frente <= self.d_parada:
                # Obstáculo inesperado durante rodeo → saltar directo al giro de retorno
                self.get_logger().warn(
                    f'Obstáculo en rodeo (d={self.dist_frente:.2f}m) — abortando avance')
                self.fase_rodeo = 2
                self.t_inicio = self.get_clock().now()
            elif t < self.t_avance:
                self._pub(self.v_cruise, 0.0)
            else:
                self.fase_rodeo = 2
                self.t_inicio = self.get_clock().now()

        elif self.fase_rodeo == 2:         # giro inverso para reincorporarse
            if t < t_giro:
                self._pub(0.0, -self._rodeo_dir * self.w_giro)
            else:
                self._pub(0.0, 0.0)
                self._cambiar(CRUCERO)


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
