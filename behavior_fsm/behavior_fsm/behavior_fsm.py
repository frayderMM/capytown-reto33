#!/usr/bin/env python3
"""
behavior_fsm.py — PARTE B: "El Guardián" (versión corregida y completa).

Un SOLO nodo hace percepción + control (antes wall_follower y behavior_fsm
procesaban el mismo /scan por separado, en marcos distintos — wall_follower
ni siquiera rotaba lidar_front_deg — y se coordinaban por /lateral_correction
con carreras de tiempo; esa inconsistencia de marcos era la causa de los
giros erráticos).

FSM exigida por el reto, restaurada:

  CRUCERO ── caja al frente ──▶ CAJA_DETECTADA ─▶ PARAR ─▶ ESPERAR_3S ─▶ RODEAR ─▶ CRUCERO
      │
      └── pared/esquina al frente ──▶ GIRAR_ESQUINA (90° izq) ──▶ CRUCERO

· CRUCERO sigue la pared DERECHA (PD distancia + alineación angular, con
  limitador de tasa de cambio de w — lección aprendida — y velocidad
  adaptativa según espacio al frente).
· La clasificación caja/esquina vuelve a ser confiable con el Split-and-Merge
  corregido (bug del [1:]) y se estabiliza con un VOTO por mayoría sobre los
  últimos N barridos: no se decide con un solo scan ruidoso.
· RODEAR: gira +45° (izquierda), avanza en diagonal para librar la caja,
  gira −45°, avanza recto hasta pasarla; al volver a CRUCERO el PD lo
  re-pega solo a la pared derecha con el mismo umbral de siempre. Distancias
  y giros por /odom_raw (fallback por tiempo si no hay odometría).
· GIRAR_ESQUINA: 90° a la IZQUIERDA (lazo antihorario) con cooldown para no
  encadenar giros. Nunca gira a la derecha en esquinas ni retrocede →
  el recorrido no puede invertirse ("no regresa").
· EMERGENCIA omnidireccional con los offsets reales del chasis
  (15 cm frente / 10 cm atrás / 8 cm lados): si algo invade el footprint
  inflado, se detiene y rota alejándose; si persiste, lo trata como caja.
· Cono trasero excluido (cable del LiDAR) y frente en raw=180° (MS200).

Publica /fsm_state (String) y /guardian/debug (String JSON) para lidar_viz.py.

ESAN - Robótica de Móviles 2026-I | Proyecto CapyTown
"""

import json
import math
from collections import Counter, deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

from behavior_fsm import percepcion as pc

# Estados
CRUCERO        = 'CRUCERO'
CAJA_DETECTADA = 'CAJA_DETECTADA'
PARAR          = 'PARAR'
ESPERAR_3S     = 'ESPERAR_3S'
RODEAR         = 'RODEAR'
GIRAR_ESQUINA  = 'GIRAR_ESQUINA'
EMERGENCIA     = 'EMERGENCIA'


class Guardian(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ─────────────────────────────────────────────────────
        # Orientación del sensor y zonas ignoradas
        self.declare_parameter('lidar_front_deg', 180.0)   # frente del MS200 Yahboom
        self.declare_parameter('excluir_atras_deg', 60.0)  # cable/soporte del LiDAR
        self.declare_parameter('remove_min_deg', float('nan'))
        self.declare_parameter('remove_max_deg', float('nan'))
        self.declare_parameter('rango_max', 3.5)
        # Offsets LiDAR → borde físico del robot
        self.declare_parameter('offset_frente', 0.15)
        self.declare_parameter('offset_atras',  0.10)
        self.declare_parameter('offset_lados',  0.08)
        # Clustering / Split-and-Merge / clasificación
        self.declare_parameter('salto_dist', 0.12)
        self.declare_parameter('salto_idx', 5)
        self.declare_parameter('umbral_split', 0.04)
        self.declare_parameter('min_puntos', 4)
        self.declare_parameter('lado_caja_max', 0.32)
        self.declare_parameter('min_long_pared', 0.45)
        self.declare_parameter('cos_lateral_min', 0.55)
        self.declare_parameter('votos_clase', 5)       # barridos para el voto por mayoría
        # Seguimiento de pared derecha
        self.declare_parameter('dist_pared', 0.15)     # m holgura LADO robot→pared
        self.declare_parameter('Kp', 1.4)
        self.declare_parameter('Kd', 0.20)
        self.declare_parameter('Ka', 1.0)
        self.declare_parameter('max_w', 0.8)
        self.declare_parameter('max_delta_w', 0.15)    # rad/s por ciclo (anti-zigzag)
        # FSM / velocidades — distancias frontales POST-OFFSET (borde real)
        self.declare_parameter('vel_crucero', 0.15)
        self.declare_parameter('vel_min', 0.07)
        self.declare_parameter('w_giro', 0.7)
        self.declare_parameter('vel_giro_arco', 0.04)
        self.declare_parameter('dist_alerta', 0.35)    # entra a CAJA_DETECTADA
        self.declare_parameter('dist_parada', 0.17)    # se detiene (reto exige ≥0.15)
        self.declare_parameter('dist_esquina', 0.20)   # borde→pared frontal p/ girar
        self.declare_parameter('dist_emergencia', 0.04)
        self.declare_parameter('espera_seg', 3.0)
        self.declare_parameter('ang_rodeo_deg', 45.0)
        self.declare_parameter('avance_diag', 0.38)
        self.declare_parameter('avance_paralelo', 0.55)
        self.declare_parameter('cooldown_esquina', 2.0)
        self.declare_parameter('topic_odom', '/odom_raw')

        g = lambda n: self.get_parameter(n).value
        self.front_rad = math.radians(g('lidar_front_deg'))
        self.atras_rad = math.radians(g('excluir_atras_deg')) / 2.0
        rm_min, rm_max = g('remove_min_deg'), g('remove_max_deg')
        self.rm_min = math.radians(rm_min) if not math.isnan(rm_min) else None
        self.rm_max = math.radians(rm_max) if not math.isnan(rm_max) else None
        self.rango_max = g('rango_max')
        self.off_f, self.off_a, self.off_l = (g('offset_frente'),
                                              g('offset_atras'),
                                              g('offset_lados'))
        self.salto_dist   = g('salto_dist')
        self.salto_idx    = int(g('salto_idx'))
        self.umbral_split = g('umbral_split')
        self.min_puntos   = int(g('min_puntos'))
        self.lado_caja    = g('lado_caja_max')
        self.min_pared    = g('min_long_pared')
        self.cos_lat      = g('cos_lateral_min')
        self.n_votos      = int(g('votos_clase'))
        self.d_obj_lidar  = g('dist_pared') + self.off_l  # objetivo LiDAR→pared
        self.Kp, self.Kd, self.Ka = g('Kp'), g('Kd'), g('Ka')
        self.max_w, self.max_dw   = g('max_w'), g('max_delta_w')
        self.v_cru, self.v_min, self.w_giro = (g('vel_crucero'), g('vel_min'),
                                               g('w_giro'))
        self.v_arco = g('vel_giro_arco')
        self.d_alerta   = g('dist_alerta')
        self.d_parada   = g('dist_parada')
        self.d_esquina  = g('dist_esquina')
        self.d_emerg    = g('dist_emergencia')
        self.espera     = g('espera_seg')
        self.ang_rodeo  = math.radians(g('ang_rodeo_deg'))
        self.av_diag    = g('avance_diag')
        self.av_par     = g('avance_paralelo')
        self.cooldown   = g('cooldown_esquina')

        # ── Estado de percepción (lo escribe cb_scan, lo lee loop) ─────────
        self.d_frente = float('inf')
        self.clase_frente = None
        self.votos = deque(maxlen=self.n_votos)   # voto por mayoría de clase
        self.pared_der = None
        self.punto_fp = None
        self.d_izq = self.d_der = float('inf')
        self.clusters = []

        # ── Odometría / censo (de box_detector) ────────────────────────────
        self.pose = None
        self.trail = []
        self.cajas_vivas = []
        self.cajas_fijas = []

        # ── FSM ─────────────────────────────────────────────────────────────
        self.estado = CRUCERO
        self.fase = 0
        self.t0 = self.get_clock().now()
        self.yaw0 = 0.0
        self.pos0 = (0.0, 0.0)
        self.t_fin_esquina = self.get_clock().now()
        self._err_prev, self._w_prev = 0.0, 0.0
        self._t_pd = self.get_clock().now()

        # ── ROS I/O ─────────────────────────────────────────────────────────
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos)
        self.create_subscription(Odometry, g('topic_odom'), self.cb_odom, qos)
        self.pub_cmd    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_estado = self.create_publisher(String, '/fsm_state', 10)
        self.pub_parada = self.create_publisher(Float32, '/parada_dist', 10)
        self.pub_debug  = self.create_publisher(String, '/guardian/debug', 10)
        self.create_timer(0.1, self.loop)

        self.get_logger().info(
            f'Guardián listo | pared der objetivo {g("dist_pared")*100:.0f} cm '
            f'| parada caja {self.d_parada*100:.0f} cm | frente LiDAR '
            f'{g("lidar_front_deg"):.0f}° | footprint 15/10/8 cm')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def cb_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        if not self.trail or math.hypot(p.x - self.trail[-1][0],
                                        p.y - self.trail[-1][1]) > 0.03:
            self.trail.append((p.x, p.y))
            self.trail = self.trail[-600:]

    def cb_scan(self, msg: LaserScan):
        pts = pc.filtrar_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max,
                              self.front_rad, self.rango_max,
                              self.atras_rad, self.rm_min, self.rm_max)
        cls = pc.analizar_scan(pts, self.salto_dist, self.salto_idx,
                               self.umbral_split, self.min_puntos,
                               self.lado_caja, self.min_pared)
        self.clusters = cls
        self.pared_der = pc.pared_derecha(cls, self.min_pared, self.cos_lat)
        (self.d_frente, clase_f, self.d_izq, self.d_der,
         self.punto_fp) = pc.frente_y_lados(pts, cls, self.off_f, self.off_l)
        self._actualizar_cajas_desde_scan(cls)

        # Voto por mayoría: la clase del frente solo cambia si la mayoría de
        # los últimos N barridos coincide — un scan ruidoso ya no decide.
        self.votos.append(clase_f)
        validos = [v for v in self.votos if v is not None]
        if validos:
            clase, n = Counter(validos).most_common(1)[0]
            self.clase_frente = clase if n >= max(2, len(self.votos) // 2) else clase_f
        else:
            self.clase_frente = None

        self._publicar_debug()

    # ── Helpers FSM ──────────────────────────────────────────────────────────
    def _t(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _cambiar(self, nuevo, fase=0):
        if nuevo != self.estado or fase != self.fase:
            self.get_logger().info(
                f'{self.estado}[{self.fase}] → {nuevo}[{fase}] '
                f'(frente={self.d_frente:.2f} m, clase={self.clase_frente})')
        self.estado, self.fase = nuevo, fase
        self.t0 = self.get_clock().now()
        if self.pose is not None:
            self.yaw0 = self.pose[2]
            self.pos0 = (self.pose[0], self.pose[1])

    def _pub(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(max(0.0, v))   # NUNCA retrocede
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String()
        s.data = self.estado
        self.pub_estado.publish(s)

    def _giro_ok(self, objetivo):
        if self.pose is not None:
            d = math.atan2(math.sin(self.pose[2] - self.yaw0),
                           math.cos(self.pose[2] - self.yaw0))
            return abs(d) >= abs(objetivo) - math.radians(3)
        return self._t() >= abs(objetivo) / self.w_giro

    def _avance_ok(self, dist):
        if self.pose is not None:
            return math.hypot(self.pose[0] - self.pos0[0],
                              self.pose[1] - self.pos0[1]) >= dist
        return self._t() >= dist / self.v_cru

    def _vel_adaptativa(self):
        """Velocidad progresiva según espacio libre al frente (post-offset)."""
        c = self.d_frente
        if c >= self.d_alerta:
            return self.v_cru
        ratio = max(0.0, min(1.0, (c - self.d_parada) /
                             (self.d_alerta - self.d_parada)))
        return self.v_min + ratio * (self.v_cru - self.v_min)

    # ── Lazo de control 10 Hz ────────────────────────────────────────────────
    def _a_odom(self, x, y):
        if self.pose is None:
            return None
        px, py, yaw = self.pose
        ox = math.cos(yaw) * x - math.sin(yaw) * y + px
        oy = math.sin(yaw) * x + math.cos(yaw) * y + py
        return ox, oy

    def _actualizar_cajas_desde_scan(self, clusters):
        vivas = []
        for cl in clusters:
            if cl['clase'] != pc.CAJA:
                continue
            cx, cy = cl['c']
            if math.hypot(cx, cy) > 2.0:
                continue
            odom = self._a_odom(cx, cy)
            if odom is None:
                continue
            ox, oy = odom
            if cx > -0.20:
                vivas.append((ox, oy))
            elif all(math.hypot(ox - fx, oy - fy) > 0.45
                     for fx, fy in self.cajas_fijas):
                self.cajas_fijas.append((ox, oy))
        self.cajas_vivas = vivas[-6:]
        self.cajas_fijas = self.cajas_fijas[-8:]

    def loop(self):
        d = self.d_frente

        if self.punto_fp is not None and self.estado != EMERGENCIA:
            self._cambiar(EMERGENCIA)

        if self.estado == CRUCERO:
            en_cd = (self.get_clock().now() - self.t_fin_esquina
                     ).nanoseconds * 1e-9 < self.cooldown
            if self.clase_frente == pc.CAJA and d < self.d_alerta:
                self._cambiar(CAJA_DETECTADA)
            elif (self.clase_frente in (pc.PARED, pc.ESQUINA)
                  and d < self.d_esquina and not en_cd):
                self._cambiar(GIRAR_ESQUINA)
            elif d < self.d_parada:
                # bloqueo sin clase clara → tratar como caja (lo más seguro:
                # detenerse ≥15 cm nunca tumba nada)
                self._pub(0.0, 0.0)
                m = Float32(); m.data = float(d)
                self.pub_parada.publish(m)
                self._cambiar(PARAR)
            else:
                self._pub(self._vel_adaptativa(), self._w_pared())

        elif self.estado == CAJA_DETECTADA:
            if d <= self.d_parada:
                self._pub(0.0, 0.0)
                m = Float32(); m.data = float(d)
                self.pub_parada.publish(m)
                self._cambiar(PARAR)
            elif d > self.d_alerta * 1.3 or self.clase_frente is None:
                self._cambiar(CRUCERO)          # se despejó
            else:
                self._pub(self.v_min, 0.0)      # acercamiento recto y lento

        elif self.estado == PARAR:
            self._pub(0.0, 0.0)
            self._cambiar(ESPERAR_3S)

        elif self.estado == ESPERAR_3S:
            self._pub(0.0, 0.0)
            if self._t() >= self.espera:
                self._cambiar(RODEAR, 0)

        elif self.estado == RODEAR:
            self._rodear()

        elif self.estado == GIRAR_ESQUINA:
            if self._giro_ok(math.pi / 2):
                self._pub(0.0, 0.0)
                self.t_fin_esquina = self.get_clock().now()
                self._w_prev = 0.0
                self._cambiar(CRUCERO)
            else:
                self._pub(0.0, self.w_giro)     # 90° IZQUIERDA (lazo CCW)

        elif self.estado == EMERGENCIA:
            if self.punto_fp is None:
                self._cambiar(CRUCERO)
            elif self._t() > 4.0:
                self._cambiar(PARAR)            # persistente → tratar como caja
            else:
                px, py = self.punto_fp
                w = self.w_giro if py <= 0 else -self.w_giro
                self._pub(0.0, 0.5 * w)         # rota alejándose, sin avanzar

    # ── Seguimiento de pared derecha (PD + alineación + rate limiter) ───────
    def _w_pared(self):
        ref = self.pared_der
        if ref is None:
            w = 0.0                              # sin referencia: recto
            self._err_prev = 0.0
        else:
            err = ref['d'] - self.d_obj_lidar   # >0: lejos
            now = self.get_clock().now()
            dt = max((now - self._t_pd).nanoseconds * 1e-9, 0.01)
            derr = (err - self._err_prev) / dt
            self._err_prev, self._t_pd = err, now
            # lejos → girar derecha (w<0); alpha≠0 → realinear paralelo
            w = -self.Kp * err - self.Kd * derr + self.Ka * ref['alpha']
            w = max(-self.max_w, min(self.max_w, w))
        # limitador de tasa de cambio (anti-zigzag, lección aprendida)
        w = max(self._w_prev - self.max_dw, min(self._w_prev + self.max_dw, w))
        self._w_prev = w
        return w

    # ── Rodeo por la izquierda ───────────────────────────────────────────────
    def _rodear(self):
        # guardia de colisión en las fases de avance
        if self.fase in (1, 3) and self.d_frente < self.d_parada:
            self._pub(0.0, 0.0)
            self._cambiar(PARAR)                 # replanifica: espera y rodea de nuevo
            return
        if self.fase == 0:                       # +45° izquierda
            if self._giro_ok(self.ang_rodeo):
                self._cambiar(RODEAR, 1)
            else:
                self._pub(self.v_arco, self.w_giro)
        elif self.fase == 1:                     # diagonal: libra la caja
            if self._avance_ok(self.av_diag):
                self._cambiar(RODEAR, 2)
            else:
                self._pub(self.v_cru, 0.0)
        elif self.fase == 2:                     # −45°: queda paralelo
            if self._giro_ok(self.ang_rodeo):
                self._cambiar(RODEAR, 3)
            else:
                self._pub(self.v_arco, -self.w_giro)
        elif self.fase == 3:                     # recto: pasa la caja
            if self._avance_ok(self.av_par):
                self._w_prev = 0.0
                self._cambiar(CRUCERO)           # el PD lo re-pega a la derecha
            else:
                self._pub(self.v_cru, self._w_pared())

    # ── Debug JSON para lidar_viz.py ─────────────────────────────────────────
    def _publicar_debug(self):
        segs = []
        for cl in self.clusters:
            for s in cl['segs']:
                segs.append({'x1': round(s['p1'][0], 3), 'y1': round(s['p1'][1], 3),
                             'x2': round(s['p2'][0], 3), 'y2': round(s['p2'][1], 3),
                             'lon': round(s['lon'], 3), 'clase': cl['clase']})
        data = {
            'estado': self.estado, 'fase': self.fase,
            'd_frente': None if not math.isfinite(self.d_frente)
                        else round(self.d_frente, 3),
            'clase_frente': self.clase_frente,
            'pared_der': (None if self.pared_der is None else
                          {'d': round(self.pared_der['d'], 3),
                           'alpha_deg': round(math.degrees(
                               self.pared_der['alpha']), 1)}),
            'd_izq': None if not math.isfinite(self.d_izq) else round(self.d_izq, 3),
            'd_der': None if not math.isfinite(self.d_der) else round(self.d_der, 3),
            'pose': None if self.pose is None else [round(v, 3) for v in self.pose],
            'trail': [[round(x, 3), round(y, 3)] for x, y in self.trail[-400:]],
            'cajas_vivas': [[round(x, 3), round(y, 3)]
                            for x, y in self.cajas_vivas],
            'cajas_fijas': [[round(x, 3), round(y, 3)]
                            for x, y in self.cajas_fijas],
            'segs': segs,
        }
        m = String()
        m.data = json.dumps(data)
        self.pub_debug.publish(m)


def main(args=None):
    rclpy.init(args=args)
    nodo = Guardian()
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
