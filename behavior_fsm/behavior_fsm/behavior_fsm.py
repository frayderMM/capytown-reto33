#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Gap navigation reactivo:
  CRUCERO → avanza con velocidad proporcional a la distancia frontal.
  PARAR   → para 0.3 s al detectar obstaculo.
  EVADIR  → gira dinamicamente hacia el angulo con mayor espacio abierto
             (donde el LiDAR no ve puntos, o los ve muy lejos).
             Cuando el frente queda libre, vuelve a CRUCERO.

No hay angulo fijo ni tiempo fijo de rodeo.
El robot sigue el hueco hasta salir del obstaculo.

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

from box_detector import lidar_utils as lu


CRUCERO = 'CRUCERO'
PARAR   = 'PARAR'
EVADIR  = 'EVADIR'
RESCATE = 'RESCATE'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  45.0)
        self.declare_parameter('sector_lateral_lo',   60.0)
        self.declare_parameter('sector_lateral_hi',  120.0)
        self.declare_parameter('dist_alerta',          0.45)
        self.declare_parameter('dist_parada',          0.18)
        self.declare_parameter('vel_crucero',          0.18)
        self.declare_parameter('vel_min',              0.05)
        self.declare_parameter('vel_giro',             0.50)
        self.declare_parameter('pausa_parada',         0.30)
        self.declare_parameter('Kgap',                 1.2)   # rad/s por radian de gap
        self.declare_parameter('gap_sector_deg',       90.0)  # ventana de busqueda del hueco
        self.declare_parameter('t_evasion_max',        8.0)   # timeout de seguridad
        self.declare_parameter('dist_emergencia',      0.12)  # stop total si algo esta a < X m

        # --- Recuperacion (RESCATE: barrido + retroceso si EVADIR se atasca) ---
        self.declare_parameter('topic_odom',                '/odom')
        self.declare_parameter('gap_fallback_deg',           15.0)
        self.declare_parameter('barrido_max_deg',            300.0)
        self.declare_parameter('barrido_t_max',               10.0)
        self.declare_parameter('vel_retroceso',                0.08)
        self.declare_parameter('retroceso_dist',               0.25)
        self.declare_parameter('retroceso_t_max',               3.0)
        self.declare_parameter('dist_seguridad_retroceso',      0.20)

        self.front_rad  = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector     = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.lat_lo     = math.radians(self.get_parameter('sector_lateral_lo').value)
        self.lat_hi     = math.radians(self.get_parameter('sector_lateral_hi').value)
        self.d_alerta   = self.get_parameter('dist_alerta').value
        self.d_parada   = self.get_parameter('dist_parada').value
        self.v_cruise   = self.get_parameter('vel_crucero').value
        self.v_min      = self.get_parameter('vel_min').value
        self.w_giro     = self.get_parameter('vel_giro').value
        self.t_pausa    = self.get_parameter('pausa_parada').value
        self.Kgap       = self.get_parameter('Kgap').value
        self.gap_sector = math.radians(self.get_parameter('gap_sector_deg').value)
        self.t_ev_max   = self.get_parameter('t_evasion_max').value
        self.d_emerg    = self.get_parameter('dist_emergencia').value

        self.topic_odom    = self.get_parameter('topic_odom').value
        self.gap_fallback  = math.radians(self.get_parameter('gap_fallback_deg').value)
        self.barrido_max   = math.radians(self.get_parameter('barrido_max_deg').value)
        self.barrido_t_max = self.get_parameter('barrido_t_max').value
        self.vel_retroceso = self.get_parameter('vel_retroceso').value
        self.retroceso_dist = self.get_parameter('retroceso_dist').value
        self.retroceso_t_max = self.get_parameter('retroceso_t_max').value
        self.dist_seg_retro = self.get_parameter('dist_seguridad_retroceso').value

        # ── Estado ────────────────────────────────────────────────────────
        self.estado   = CRUCERO
        self.t_inicio = self.get_clock().now()

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente = float('inf')
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self.dist_atras  = float('inf')
        self._gap_ang    = 0.0   # ángulo al mayor espacio abierto
        self._w_lateral  = 0.0

        # ── Odometría (para RESCATE) ─────────────────────────────────────
        self.pose       = (0.0, 0.0, 0.0)   # (x, y, yaw) en marco odom
        self.tengo_odom = False

        # ── Sub-estado de RESCATE ────────────────────────────────────────
        self._rescate_fase        = None   # 'BARRIDO' | 'RETROCEDER' | None
        self._rescate_sentido     = 0      # +1/-1, sentido de giro en BARRIDO
        self._rescate_yaw_prev    = 0.0
        self._rescate_yaw_acum    = 0.0
        self._rescate_xy0         = (0.0, 0.0)
        self._rescate_fase_t0     = self.get_clock().now()

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        _qos_odom = QoSProfile(depth=10)
        _qos_odom.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan',               self.cb_scan,  _qos)
        self.create_subscription(Float32,   '/lateral_correction', self._cb_lat,   10)
        self.create_subscription(Odometry,  self.topic_odom,       self.cb_odom, _qos_odom)
        self.pub_cmd    = self.create_publisher(Twist,   '/cmd_vel',     10)
        self.pub_estado = self.create_publisher(String,  '/fsm_state',   10)
        self.pub_parada = self.create_publisher(Float32, '/parada_dist', 10)
        self.create_timer(0.1, self.loop_control)

        self.get_logger().info('BehaviorFSM listo — gap navigation reactivo')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def cb_scan(self, msg: LaserScan):
        d_f = d_l = d_r = d_b = float('inf')
        rear_rad = self.front_rad + math.pi

        # Para el hueco: acumular rangos por bucket de 10°
        bucket_sum   = {}
        bucket_count = {}

        for i, r in enumerate(msg.ranges):
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            abs_af = abs(af)
            valid  = math.isfinite(r) and msg.range_min <= r <= msg.range_max

            # Sector frontal → dist_frente
            if abs_af <= self.sector and valid:
                d_f = min(d_f, r)

            # Sector trasero (espejo del frontal) → dist_atras, para RESCATE
            ab = math.atan2(math.sin(raw - rear_rad), math.cos(raw - rear_rad))
            if abs(ab) <= self.sector and valid:
                d_b = min(d_b, r)

            # Sectores laterales → dist_izq / dist_der
            if self.lat_lo <= abs_af <= self.lat_hi and valid:
                if af > 0:
                    d_l = min(d_l, r)
                else:
                    d_r = min(d_r, r)

            # Ventana de búsqueda del hueco (±gap_sector)
            if abs_af <= self.gap_sector:
                # Sin retorno LiDAR = espacio abierto → usar range_max
                r_gap  = r if valid else msg.range_max
                bucket = round(math.degrees(af) / 10.0) * 10
                bucket_sum[bucket]   = bucket_sum.get(bucket, 0.0) + r_gap
                bucket_count[bucket] = bucket_count.get(bucket, 0) + 1

        self.dist_frente = d_f
        self.dist_izq    = d_l
        self.dist_der    = d_r
        self.dist_atras  = d_b

        # Hueco = bucket con mayor rango promedio dentro de la ventana
        # Suavizado EMA para evitar oscilacion rapida de direccion en esquinas
        if bucket_count:
            best = max(bucket_sum, key=lambda k: bucket_sum[k] / bucket_count[k])
            new_gap = math.radians(best)
            self._gap_ang = 0.35 * new_gap + 0.65 * self._gap_ang

    def _cb_lat(self, msg: Float32):
        self._w_lateral = msg.data

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = lu.yaw_desde_quaternion(q.x, q.y, q.z, q.w)
        self.pose = (p.x, p.y, yaw)
        self.tengo_odom = True

    # ── Helpers ───────────────────────────────────────────────────────────
    def _pub(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String(); s.data = self.estado
        self.pub_estado.publish(s)

    def _t_estado(self) -> float:
        return (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9

    def _cambiar(self, nuevo: str):
        self.get_logger().info(
            f'{self.estado} → {nuevo}  '
            f'(frente={self.dist_frente:.2f} m  '
            f'gap={math.degrees(self._gap_ang):.0f}°)')
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def _cambiar_fase_rescate(self, nueva_fase: str):
        self.get_logger().info(f'RESCATE fase: {self._rescate_fase} → {nueva_fase}')
        self._rescate_fase    = nueva_fase
        self._rescate_fase_t0 = self.get_clock().now()

    def _t_fase_rescate(self) -> float:
        return (self.get_clock().now() - self._rescate_fase_t0).nanoseconds * 1e-9

    def _iniciar_barrido(self):
        self._rescate_sentido  = 1.0 if self.dist_izq >= self.dist_der else -1.0
        self._rescate_yaw_prev = self.pose[2]
        self._rescate_yaw_acum = 0.0
        self._cambiar_fase_rescate('BARRIDO')

    def _vel_adaptativa(self) -> float:
        d = self.dist_frente
        if d >= self.d_alerta:
            return self.v_cruise
        ratio = (d - self.d_parada) / (self.d_alerta - self.d_parada)
        return self.v_min + max(0.0, min(1.0, ratio)) * (self.v_cruise - self.v_min)

    # ── FSM principal ─────────────────────────────────────────────────────
    def loop_control(self):

        # Contingencia de colisión — override de cualquier estado
        if self.dist_frente < self.d_emerg:
            self.get_logger().warn(
                f'EMERGENCIA frente={self.dist_frente:.2f}m — stop total', throttle_duration_sec=1.0)
            self._pub(0.0, 0.0)
            return

        if self.estado == CRUCERO:
            if self.dist_frente <= self.d_parada:
                d_msg = Float32(); d_msg.data = float(self.dist_frente)
                self.pub_parada.publish(d_msg)
                self._cambiar(PARAR)
                return
            v = self._vel_adaptativa()
            w = self._w_lateral if self.dist_frente >= self.d_alerta else 0.0
            self._pub(v, w)

        elif self.estado == PARAR:
            self._pub(0.0, 0.0)
            if self._t_estado() >= self.t_pausa:
                self._cambiar(EVADIR)

        elif self.estado == EVADIR:
            # Timeout de seguridad — el gap-follow normal no encontró salida.
            # Escalar a RESCATE (barrido activo + retroceso) en vez de
            # forzar CRUCERO a ciegas, que solo redetecta el mismo bloqueo.
            if self._t_estado() > self.t_ev_max:
                self.get_logger().warn('Timeout evasion — escalando a RESCATE')
                self._cambiar(RESCATE)
                self._iniciar_barrido()
                return

            # Frente despejado Y mínimo 0.5 s girando → evasión completada.
            # Sin el mínimo, el robot entra y sale de EVADIR tan rápido que
            # oscila en las esquinas sin completar el giro.
            if self.dist_frente > self.d_alerta and self._t_estado() > 0.5:
                self._cambiar(CRUCERO)
                return

            gap = self._gap_ang
            # En esquinas el gap suavizado puede ser pequeño o ambiguo:
            # usar las distancias laterales reales para elegir sentido.
            if abs(gap) < self.gap_fallback or self.dist_frente <= self.d_parada:
                w = self.w_giro if self.dist_izq >= self.dist_der else -self.w_giro
            else:
                w = max(-self.w_giro, min(self.w_giro, self.Kgap * gap))

            # Avanzar mientras se gira hacia el hueco (bordeo), no girar en
            # el sitio y avanzar despues. La unica condicion de seguridad
            # real es que no haya nada pegado justo al frente; EMERGENCIA
            # ya cubre el caso de colision inminente por separado.
            puede_avanzar = self.dist_frente > self.d_parada
            v = self._vel_adaptativa() if puede_avanzar else 0.0

            self._pub(v, w)

        elif self.estado == RESCATE:
            self._loop_rescate()

    # ── RESCATE: barrido activo (odometría) + retroceso controlado ─────────
    def _loop_rescate(self):
        if not self.tengo_odom:
            # Sin odometría no se puede medir giro/desplazamiento con
            # seguridad — esperar en el sitio en vez de mover a ciegas.
            self._pub(0.0, 0.0)
            return

        if self._rescate_fase == 'BARRIDO':
            self._rescate_barrido()
        elif self._rescate_fase == 'RETROCEDER':
            self._rescate_retroceder()
        else:
            # Estado inconsistente (no debería pasar) — reiniciar barrido.
            self._iniciar_barrido()

    def _rescate_barrido(self):
        # Apertura real encontrada (cb_scan se sigue actualizando mientras
        # el robot gira) → volver a EVADIR para reincorporarse con el
        # control proporcional normal.
        if self.dist_frente > self.d_alerta:
            self._cambiar(EVADIR)
            return

        # Acumular rotación real recorrida (delta por-tick, no contra una
        # referencia fija — atan2 envuelve a [-pi,pi] y subestimaría giros
        # de mas de 180°).
        yaw_actual = self.pose[2]
        d_yaw = math.atan2(math.sin(yaw_actual - self._rescate_yaw_prev),
                           math.cos(yaw_actual - self._rescate_yaw_prev))
        self._rescate_yaw_acum += abs(d_yaw)
        self._rescate_yaw_prev = yaw_actual

        if (self._rescate_yaw_acum >= self.barrido_max
                or self._t_fase_rescate() >= self.barrido_t_max):
            # Barrido agotado sin encontrar apertura → intentar retroceder.
            if self.dist_atras < self.dist_seg_retro:
                self.get_logger().warn(
                    'RESCATE: barrido agotado y atras tambien bloqueado — reintentando barrido')
                self._iniciar_barrido()
                return
            self._rescate_xy0 = (self.pose[0], self.pose[1])
            self._cambiar_fase_rescate('RETROCEDER')
            return

        self._pub(0.0, self._rescate_sentido * self.w_giro)

    def _rescate_retroceder(self):
        # El frente ya esta cubierto por la contingencia de EMERGENCIA al
        # inicio de loop_control; aqui se vigila la parte trasera, que
        # EMERGENCIA nunca chequea.
        if self.dist_atras < self.dist_seg_retro:
            self.get_logger().warn('RESCATE: retroceso abortado — obstaculo trasero')
            self._iniciar_barrido()
            return

        x0, y0 = self._rescate_xy0
        x, y, _ = self.pose
        desplazado = math.hypot(x - x0, y - y0)

        if desplazado >= self.retroceso_dist or self._t_fase_rescate() >= self.retroceso_t_max:
            self._cambiar(EVADIR)
            return

        self._pub(-self.vel_retroceso, 0.0)


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
