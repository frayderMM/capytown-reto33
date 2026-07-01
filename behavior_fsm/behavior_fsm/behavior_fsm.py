#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Control reactivo continuo. La pista es un circuito cerrado (jiron de
60cm) con paredes a ambos lados, asi que con el frente libre se sigue
la pared derecha (correccion de wall_follower.py via /lateral_correction,
objetivo 8cm) para recorrer el circuito -- no es "avanzar sin rumbo".
Cuando algo bloquea el frente (caja o esquina real del circuito -- no
se distinguen: medir el ANCHO de lo que bloquea el frente con el LiDAR
resulto poco confiable en pista y confundia una caja con una esquina,
dejando al robot forzado a girar hacia un lado sin espacio real), esa
guia se reemplaza por una unica evasion en "U":

  - velocidad: progresiva, baja segun se cierra el espacio al frente.
  - giro: con el frente libre, sigue la pared derecha. Si algo bloquea
    el frente, se evade siempre hacia el lado con espacio REAL
    disponible (prioriza izquierda -- mismo sentido en que se sigue el
    circuito -- pero cae a la derecha si a la izquierda no hay
    espacio), hasta que el frente se despeje; luego vuelve a pegarse a
    la derecha. La fuerza del giro se limita segun cuanto espacio real
    hay en el lado elegido, para nunca forzar un giro hacia un lado sin
    salida (eso dejaba al robot atascado girando casi sin avanzar).

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
        self.declare_parameter('dist_alerta',     0.33)  # m  empieza a frenar y a anticipar el giro
                                                           # (cerca de dist_obstaculo: sigue pegado
                                                           # a la derecha mientras el obstaculo esta
                                                           # lejos, solo reacciona cuando esta cerca)
        self.declare_parameter('dist_obstaculo',  0.30)  # m  giro a maxima intensidad
        self.declare_parameter('dist_emergencia', 0.04)  # m  margen real minimo antes del stop total

        # --- Velocidades ---
        self.declare_parameter('vel_crucero', 0.22)
        self.declare_parameter('vel_min',     0.08)
        self.declare_parameter('w_giro_max',  0.45)  # rad/s  maximo giro al evadir un frente bloqueado

        self.declare_parameter('t_evasion_min', 0.3)  # s  compromiso minimo en evasion
                                                        # antes de poder volver a AVANCE -- evita
                                                        # pegarse de nuevo a la derecha con el
                                                        # obstaculo apenas despejado del frente
                                                        # pero aun al lado
        self.declare_parameter('margen_salida_evasion', 0.05)  # m  extra sobre dist_alerta
                                                        # para SALIR de evasion (histeresis) --
                                                        # con el mismo umbral para entrar y
                                                        # salir, c_frente oscilando justo en
                                                        # dist_alerta hacia adentro/afuera de
                                                        # evasion en ciclos sucesivos

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

        self.t_evasion_min = self.get_parameter('t_evasion_min').value
        self.margen_salida = self.get_parameter('margen_salida_evasion').value

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente = float('inf')  # cono ancho frontal
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self.dist_min    = float('inf')  # minimo global, todos los angulos
        self._w_lateral  = 0.0           # correccion de wall_follower (seguir pared derecha)

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan',               self.cb_scan, _qos)
        self.create_subscription(Float32,   '/lateral_correction', self._cb_lat, 10)
        self.pub_cmd    = self.create_publisher(Twist,   '/cmd_vel',     10)
        self.pub_estado = self.create_publisher(String,  '/fsm_state',   10)
        self.pub_parada = self.create_publisher(Float32, '/parada_dist', 10)
        self.create_timer(0.1, self.loop_control)

        self._en_evasion = False  # para publicar /parada_dist solo al entrar a la zona de evasion
        self._lado_evasion = 1.0  # lado de evasion, fijado UNA vez al entrar
        self._t_evasion_inicio = self.get_clock().now()

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

    def _cb_lat(self, msg: Float32):
        self._w_lateral = msg.data

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

        # Peligro (frente en zona de alerta O costado a menos de d_emerg):
        # si NO se estaba ya evadiendo, decide el LADO una sola vez aqui
        # (segun cual costado tiene espacio real), y queda fijo para las
        # tres ramas siguientes (EMERGENCIA, RECALIBRAR, evasion normal).
        # Antes, EMERGENCIA/RECALIBRAR reseteaban el episodio de evasion
        # en cada disparo -- y durante un giro real en un pasillo angosto
        # disparan seguido -- asi que el giro nunca llegaba a completarse
        # limpio, se reiniciaba una y otra vez (la causa real de las
        # trayectorias en espiral / "360").
        peligro = c_frente < self.d_alerta or c_min < self.d_emerg
        if peligro and not self._en_evasion:
            self._lado_evasion = 1.0 if c_izq >= (self.d_emerg + 0.05) else -1.0
            self._en_evasion = True
            self._t_evasion_inicio = self.get_clock().now()
            d_msg = Float32(); d_msg.data = float(c_frente)
            self.pub_parada.publish(d_msg)

        # Emergencia de frente: no se puede seguir avanzando (chocaria de
        # lleno), pero girar EN EL SITIO (v=0, sin avanzar hacia el
        # obstaculo) hacia el lado ya comprometido no lo empeora y le da
        # una salida -- quedarse en (0,0) para siempre lo dejaba pegado
        # en el mismo lugar indefinidamente.
        if c_frente < self.d_emerg:
            w = self._lado_evasion * self.w_giro_max
            self.get_logger().warn(
                f'EMERGENCIA frente={c_frente:.2f}m — girando en el sitio para salir',
                throttle_duration_sec=1.0)
            self._pub(0.0, w, 'EMERGENCIA')
            return

        # Algo muy cerca por un costado/diagonal (frente libre): en vez de
        # parar, gira hacia el lado ya comprometido mientras sigue
        # avanzando -- se recalibra sin detenerse.
        if c_min < self.d_emerg:
            w = self._lado_evasion * self.w_giro_max
            self.get_logger().warn(
                f'cerca por un costado (margen={c_min:.2f}m) — girando hacia el lado mas libre',
                throttle_duration_sec=1.0)
            self._pub(v, w, 'RECALIBRAR')
            return

        # Giro: solo reacciona a lo que tiene perpendicular al frente (no
        # a paredes laterales paralelas al avance). Progresivo desde
        # dist_alerta hasta el maximo en dist_obstaculo. El LADO ya se
        # decidio arriba y queda fijo todo el episodio -- reevaluarlo cada
        # ciclo (10Hz) es lo que producia giros erraticos cuando c_izq/
        # c_der estaban parecidos. Prioriza izquierda (mismo sentido en
        # que se sigue el circuito); solo cae a la derecha si ahi no hay
        # espacio real de entrada. La fuerza del giro se limita segun
        # cuanto espacio REAL hay en el lado elegido (no solo que el
        # frente este bloqueado): casi sin margen ahi, se modula hasta
        # casi 0 -- nunca se fuerza un giro contra una pared sin salida.
        w = 0.0
        estado = 'AVANCE'
        t_desde_evasion = (self.get_clock().now() - self._t_evasion_inicio).nanoseconds * 1e-9
        # Compromiso minimo en evasion: si el frente se despeja muy rapido
        # (el obstaculo ya no esta al frente pero puede seguir al lado),
        # no volver de inmediato a pegarse a la derecha -- se mantiene la
        # evasion hasta pasarlo por completo (t_evasion_min) para no
        # acercarse al obstaculo que todavia esta al costado. Salida
        # SIEMPRE en cuanto el frente se despeja (mas alla de esa gracia
        # breve) -- exigir completar un angulo medido se probo y podia
        # quedarse girando muy lento e indefinidamente cuando el lado
        # elegido tenia poco espacio (factor_espacio cerca de 0), sin
        # volver nunca a pegarse a la pared derecha.
        forzar_evasion = self._en_evasion and t_desde_evasion < self.t_evasion_min

        # Histeresis: para ENTRAR a evasion basta cruzar dist_alerta (igual
        # que la deteccion de "peligro" arriba); para SALIR, ya en evasion,
        # se exige despejar un poco mas alla (dist_alerta + margen_salida) --
        # sin esto, c_frente oscilando justo en el umbral producia entradas
        # y salidas de evasion en ciclos sucesivos (zigzag).
        umbral = self.d_alerta + self.margen_salida if self._en_evasion else self.d_alerta

        if c_frente < umbral or forzar_evasion:
            if c_frente < self.d_alerta:
                ratio = max(0.0, min(1.0, (self.d_alerta - c_frente) / (self.d_alerta - self.d_obst)))
            else:
                ratio = 0.0  # solo en gracia de evasion (t_evasion_min): avanza recto

            lado = self._lado_evasion
            estado = 'EVADIR'

            c_lado = c_izq if lado > 0 else c_der
            factor_espacio = max(0.0, min(1.0, (c_lado - self.d_emerg) / (self.d_alerta - self.d_emerg)))
            w = lado * ratio * factor_espacio * self.w_giro_max
        else:
            # Frente libre, sin obstaculo que evadir: seguir la pared
            # derecha para recorrer el circuito.
            self._en_evasion = False
            w = self._w_lateral

        self.get_logger().info(
            f'c_frente={c_frente:.2f} c_izq={c_izq:.2f} c_der={c_der:.2f} '
            f'v={v:.2f} w={w:.2f}  estado={estado}',
            throttle_duration_sec=0.5)
        self._pub(v, w, estado)


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
