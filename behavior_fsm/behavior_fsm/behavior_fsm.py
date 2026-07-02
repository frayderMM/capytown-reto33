#!/usr/bin/env python3
"""
behavior_fsm.py — Guardian v4 (RC3): motor de movimiento + detección de
caja/pared por tamaño para el panel de diagnóstico (lidar_viz.py).

Estados: CRUCERO → GIRO → RODEO → CRUCERO

El motor de movimiento (control, STOP de seguridad, GIRO/RODEO) es el de
RC3 (2)/RC3/behavior_fsm/behavior_fsm/behavior_fsm.py, sin cambios: gira
hacia el lado con MÁS espacio disponible en cada momento (no siempre hacia
la izquierda) y RODEO avanza recto hasta que el frente se despeja, con
mínimos/máximos de tiempo como salvavidas.

dist_frente y dist_izq_raw/dist_der_raw (mínimo crudo, sin filtrar) se
miden localmente sobre /scan: son la base del STOP de seguridad y deben
ver CUALQUIER objeto cercano. dist_izq / dist_der llegan del nodo
wall_follower (RANSAC sobre las paredes laterales, vía /dist_izq y
/dist_der) y se usan solo para el tracking de crucero y la elección de
lado en GIRO — RANSAC descarta a propósito los objetos que no son pared
(p.ej. una caja pegada al costado) como outliers, por lo que NO sirven
para detectar un choque lateral inminente.

Aparte del motor de movimiento, este nodo corre en paralelo la
clasificación caja/pared de percepcion.py (Split-and-Merge por tamaño de
línea: ≤ lado_caja_linea → caja, mayor → pared) únicamente para poblar
/guardian/debug (clase_frente, segmentos coloreados, cajas vivas, trail)
que consume lidar_viz.py. Esa clasificación NO decide el movimiento — el
motor de RC3 reacciona solo a distancias (dist_frente/dist_izq/dist_der),
igual que en RC3 (2).

Tópicos debug: /dist_frente /dbg/dist_izq_fsm /dbg/dist_der_fsm
               /dbg/w_front /dbg/w_der /dbg/w_izq /dbg/w_total
               /guardian/debug (JSON para lidar_viz.py)
Suscritos:     /dist_izq /dist_der (de wall_follower, RANSAC)
               /odom_raw (trail + censo de cajas vivas para el panel)
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

from behavior_fsm import percepcion as pc


CRUCERO = 'CRUCERO'
GIRO    = 'GIRO'
RODEO   = 'RODEO'


class Guardian(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros de movimiento (RC3) ───────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  30.0)
        self.declare_parameter('t_espera_inicio',     10.0)  # s  cuenta regresiva tras ENTER

        self.declare_parameter('d_stop_front',   0.14)
        self.declare_parameter('d_stop_lateral', 0.06)
        self.declare_parameter('d_giro',         0.30)
        self.declare_parameter('d_front_inicio', 0.40)

        self.declare_parameter('target_der', 0.17)   # robot consistente a 17-20cm
        self.declare_parameter('Kder',        2.6)
        self.declare_parameter('Kd_der',      0.3)
        self.declare_parameter('d_izq_min',  0.15)
        self.declare_parameter('Kizq',        4.0)
        self.declare_parameter('Kfront',      2.0)

        self.declare_parameter('vel_crucero',      0.10)
        self.declare_parameter('vel_maniobra',     0.07)
        self.declare_parameter('vel_giro_gradual', 0.55)
        self.declare_parameter('max_w',            0.60)

        self.declare_parameter('t_giro_min',        0.8)
        self.declare_parameter('t_giro_max',        4.0)
        self.declare_parameter('d_lado_salida_giro', 0.20)
        self.declare_parameter('k_urgencia_giro',   1.7)
        self.declare_parameter('t_rodeo_min',       0.4)
        self.declare_parameter('t_rodeo_max',       1.2)
        self.declare_parameter('t_cooldown',        2.0)
        self.declare_parameter('t_recuperacion',    1.5)

        # ── Parámetros de percepción / clasificación (solo para el panel) ─
        self.declare_parameter('excluir_atras_deg', 60.0)
        self.declare_parameter('rango_max_clasif', 3.5)
        self.declare_parameter('salto_dist', 0.12)
        self.declare_parameter('salto_idx', 5)
        self.declare_parameter('umbral_split', 0.04)
        self.declare_parameter('min_puntos', 4)
        self.declare_parameter('lado_caja_max', 0.32)
        self.declare_parameter('lado_caja_linea', 0.22)
        self.declare_parameter('offset_frente', 0.15)
        self.declare_parameter('offset_atras', 0.10)
        self.declare_parameter('offset_lados', 0.08)
        self.declare_parameter('topic_odom', '/odom_raw')

        # ── Cargar movimiento ─────────────────────────────────────────────
        self.front_rad = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector    = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.t_espera_inicio = self.get_parameter('t_espera_inicio').value
        self._reload_params()
        self.add_on_set_parameters_callback(self._on_params)

        # ── Cargar percepción / clasificación ─────────────────────────────
        g = lambda n: self.get_parameter(n).value
        self.atras_rad       = math.radians(g('excluir_atras_deg')) / 2.0
        self.rango_max_clasif = g('rango_max_clasif')
        self.salto_dist      = g('salto_dist')
        self.salto_idx       = int(g('salto_idx'))
        self.umbral_split    = g('umbral_split')
        self.min_puntos      = int(g('min_puntos'))
        self.lado_caja       = g('lado_caja_max')
        self.lado_caja_linea = g('lado_caja_linea')
        self.off_f = g('offset_frente')
        self.off_a = g('offset_atras')
        self.off_l = g('offset_lados')

        # ── Estado FSM (RC3) ──────────────────────────────────────────────
        self.estado        = CRUCERO
        self.t_inicio      = self.get_clock().now()
        self.t_ultimo_giro = -float('inf')
        self.dir_giro      = 1.0   # +1 = izquierda (w>0), -1 = derecha (w<0)
        self.w_giro_efectivo = 0.0  # magnitud del giro, congelada al entrar a GIRO

        # ── PD tracking pared derecha ─────────────────────────────────────
        self._err_der_prev = 0.0
        self._t_der_prev   = self.get_clock().now()

        # ── Sensores de movimiento ────────────────────────────────────────
        self.dist_frente = float('inf')
        self.dist_atras  = float('inf')
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self.dist_izq_raw = float('inf')
        self.dist_der_raw = float('inf')

        # ── Percepción / clasificación (solo para el panel) ───────────────
        self.clusters = []
        self.clase_frente = None
        self.accion = 'AVANZANDO'

        # ── Odometría / censo (para el panel derecho) ─────────────────────
        self.pose = None
        self.trail = []
        self.cajas_vivas = []

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.cb_scan, _qos)
        self.create_subscription(Float32, '/dist_izq', self._cb_dist_izq, _qos)
        self.create_subscription(Float32, '/dist_der', self._cb_dist_der, _qos)
        self.create_subscription(Odometry, g('topic_odom'), self.cb_odom, _qos)
        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel',   10)
        self.pub_estado = self.create_publisher(String, '/fsm_state', 10)
        self.pub_debug  = self.create_publisher(String, '/guardian/debug', 10)

        # /dist_izq y /dist_der los publica wall_follower (RANSAC); este nodo
        # solo los consume. Republicamos lo que efectivamente usa la FSM bajo
        # /dbg/ para no crear un segundo publicador sobre el mismo tópico.
        self.pub_df  = self.create_publisher(Float32, '/dist_frente',  10)
        self.pub_dl  = self.create_publisher(Float32, '/dbg/dist_izq_fsm', 10)
        self.pub_dr  = self.create_publisher(Float32, '/dbg/dist_der_fsm', 10)
        self.pub_wf  = self.create_publisher(Float32, '/dbg/w_front',  10)
        self.pub_wd  = self.create_publisher(Float32, '/dbg/w_der',    10)
        self.pub_wi  = self.create_publisher(Float32, '/dbg/w_izq',    10)
        self.pub_wt  = self.create_publisher(Float32, '/dbg/w_total',  10)

        self.create_timer(0.05, self.loop_control)

        self.get_logger().info(
            f'Guardian v4 (RC3)  giro<{self.d_giro}m  front_ini={self.d_front_ini}m'
            f'  target_der={self.target_der}m  Kder={self.Kder}'
            f'  t_rodeo=[{self.t_rodeo_min},{self.t_rodeo_max}]s (salida por sensor)')

    def _reload_params(self):
        self.d_stop_front      = self.get_parameter('d_stop_front').value
        self.d_stop_lat        = self.get_parameter('d_stop_lateral').value
        self.d_giro            = self.get_parameter('d_giro').value
        self.d_front_ini       = self.get_parameter('d_front_inicio').value
        self.target_der        = self.get_parameter('target_der').value
        self.Kder               = self.get_parameter('Kder').value
        self.Kd_der             = self.get_parameter('Kd_der').value
        self.d_izq_min         = self.get_parameter('d_izq_min').value
        self.Kizq              = self.get_parameter('Kizq').value
        self.Kfront            = self.get_parameter('Kfront').value
        self.v_cruise          = self.get_parameter('vel_crucero').value
        self.v_maniobra        = self.get_parameter('vel_maniobra').value
        self.w_giro            = self.get_parameter('vel_giro_gradual').value
        self.max_w             = self.get_parameter('max_w').value
        self.t_giro_min        = self.get_parameter('t_giro_min').value
        self.t_giro_max        = self.get_parameter('t_giro_max').value
        self.d_lado_salida_giro = self.get_parameter('d_lado_salida_giro').value
        self.k_urgencia_giro   = self.get_parameter('k_urgencia_giro').value
        self.t_rodeo_min       = self.get_parameter('t_rodeo_min').value
        self.t_rodeo_max       = self.get_parameter('t_rodeo_max').value
        self.t_cooldown        = self.get_parameter('t_cooldown').value
        self.t_recuperacion    = self.get_parameter('t_recuperacion').value

    def _on_params(self, params):
        self._reload_params()
        self.get_logger().info(f'Params: {[p.name for p in params]}')
        return SetParametersResult(successful=True)

    # ── Odometría / trail / censo vivo (solo para el panel derecho) ──────────
    def cb_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        if not self.trail or math.hypot(p.x - self.trail[-1][0],
                                        p.y - self.trail[-1][1]) > 0.03:
            self.trail.append((p.x, p.y))
            self.trail = self.trail[-600:]

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
            vivas.append(odom)
        self.cajas_vivas = vivas[-6:]

    def cb_scan(self, msg: LaserScan):
        # Frente + mínimo crudo lateral: se miden aquí mismo, sin depender de
        # otro nodo ni de RANSAC, porque son la base del STOP de seguridad y
        # necesitan ver CUALQUIER objeto cercano (pared o caja), no solo la
        # pared "limpia" que RANSAC reporta para el tracking de crucero.
        d_f = d_atras = float('inf')
        d_l_raw = d_r_raw = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            abs_af = abs(af)
            if abs_af <= self.sector:
                d_f = min(d_f, r)
            elif abs_af >= math.pi - self.sector:
                d_atras = min(d_atras, r)
            elif af > 0:
                d_l_raw = min(d_l_raw, r)
            else:
                d_r_raw = min(d_r_raw, r)
        # d_f/d_atras/d_l_raw/d_r_raw son crudas desde el LiDAR; el borde
        # físico del robot está más adelante/afuera del sensor
        # (offset_frente=15cm, offset_atras=10cm, offset_lados=8cm). Sin
        # restarlo, "predecir" el choque con d_stop_front=0.14 en realidad
        # dispara cuando el borde YA está a -1cm (pasado) — no es
        # predicción, es reacción tardía.
        self.dist_frente   = d_f - self.off_f
        self.dist_atras    = d_atras - self.off_a
        self.dist_izq_raw  = d_l_raw - self.off_l
        self.dist_der_raw  = d_r_raw - self.off_l

        # Clasificación caja/pared (percepcion.py) — solo para el panel de
        # diagnóstico; NO decide el movimiento (eso lo hace loop_control con
        # dist_frente/dist_izq/dist_der, igual que en RC3).
        pts = pc.filtrar_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max,
                              self.front_rad, self.rango_max_clasif, self.atras_rad)
        self.clusters = pc.analizar_scan(pts, self.salto_dist, self.salto_idx,
                                         self.umbral_split, self.min_puntos,
                                         self.lado_caja)
        _, clase_f, _, _, _, _ = pc.frente_y_lados(
            pts, self.clusters, self.off_f, self.off_l)
        self.clase_frente = clase_f
        self._actualizar_cajas_desde_scan(self.clusters)

        self._publicar_debug()

    def _cb_dist_izq(self, msg: Float32):
        self.dist_izq = msg.data

    def _cb_dist_der(self, msg: Float32):
        self.dist_der = msg.data

    def _pub(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String(); s.data = self.estado
        self.pub_estado.publish(s)
        if self.estado == GIRO:
            self.accion = 'GIRANDO'
        elif self.estado == RODEO:
            self.accion = 'BORDEANDO_OBSTACULO'
        elif abs(cmd.angular.z) > 0.08 and cmd.linear.x > 0.01:
            self.accion = 'CORRIGIENDO_DERECHA'
        elif abs(cmd.angular.z) > 0.08:
            self.accion = 'GIRANDO'
        elif cmd.linear.x > 0.01:
            self.accion = 'AVANZANDO'
        else:
            self.accion = 'DETENIDO'

    def _pub_dbg(self, wf, wd, wi, wt):
        def f32(x):
            m = Float32()
            m.data = float(x) if math.isfinite(x) else -999.0
            return m
        self.pub_df.publish(f32(self.dist_frente))
        self.pub_dl.publish(f32(self.dist_izq))
        self.pub_dr.publish(f32(self.dist_der))
        self.pub_wf.publish(f32(wf))
        self.pub_wd.publish(f32(wd))
        self.pub_wi.publish(f32(wi))
        self.pub_wt.publish(f32(wt))

    def _t_estado(self) -> float:
        return (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9

    def _w_der_pd(self) -> float:
        """PD de tracking a la pared derecha. El término derivativo amortigua
        el overshoot que trae subir Kder para pegarse más rápido al target."""
        error = self.dist_der - self.target_der
        now   = self.get_clock().now()
        dt    = max((now - self._t_der_prev).nanoseconds * 1e-9, 0.01)
        d_err = (error - self._err_der_prev) / dt
        self._err_der_prev = error
        self._t_der_prev   = now
        return -(self.Kder * error + self.Kd_der * d_err)

    def _cambiar(self, nuevo: str):
        self.get_logger().info(
            f'{self.estado}→{nuevo}  '
            f'f={self.dist_frente:.2f}  l={self.dist_izq:.2f}  r={self.dist_der:.2f}'
            f'  clase_frente={self.clase_frente}')
        # Marca cooldown al salir de GIRO o RODEO hacia CRUCERO
        if self.estado in (GIRO, RODEO) and nuevo == CRUCERO:
            self.t_ultimo_giro = self.get_clock().now().nanoseconds * 1e-9
            # Resetea la derivada: si no, el primer tick calcula d_err sobre
            # un salto acumulado durante todo GIRO+RODEO (spike falso).
            if math.isfinite(self.dist_der):
                self._err_der_prev = self.dist_der - self.target_der
            self._t_der_prev = self.get_clock().now()
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def _calibrar_tras_stop(self):
        """Calibración tras el STOP: en vez de quedarse congelado hasta que
        el obstáculo se mueva solo, arranca un GIRO fresco de una vez —
        misma elección de lado que el disparo normal de GIRO (más espacio
        disponible). Esto es la ÚNICA diferencia con la lógica de avance
        original (CRUCERO/GIRO/RODEO intactos); el STOP ya no es un
        callejón sin salida."""
        if math.isfinite(self.dist_izq) and math.isfinite(self.dist_der):
            self.dir_giro = 1.0 if self.dist_izq >= self.dist_der else -1.0
        elif math.isfinite(self.dist_der):
            self.dir_giro = -1.0
        else:
            self.dir_giro = 1.0
        self.w_giro_efectivo = max(-self.max_w, min(self.max_w,
            self.dir_giro * self.w_giro))
        self._cambiar(GIRO)

    def loop_control(self):

        # ── PRIORIDAD 1: STOP absoluto ────────────────────────────────────
        if self.dist_frente < self.d_stop_front:
            self.get_logger().warn(
                f'PARA frente={self.dist_frente:.3f}m — calibrando', throttle_duration_sec=0.4)
            self._calibrar_tras_stop()
        # Usa el mínimo crudo (dist_izq_raw/dist_der_raw), NO el de RANSAC:
        # RANSAC descarta a propósito objetos que no son pared (p.ej. una
        # caja pegada al costado) como outliers, y ese es justo el caso que
        # este STOP tiene que detectar.
        elif math.isfinite(self.dist_izq_raw) and self.dist_izq_raw < self.d_stop_lat:
            self.get_logger().warn(
                f'PARA izq={self.dist_izq_raw:.3f}m — calibrando', throttle_duration_sec=0.4)
            self._calibrar_tras_stop()
        elif math.isfinite(self.dist_der_raw) and self.dist_der_raw < self.d_stop_lat:
            # El GIRO puede ir hacia cualquier lado (no solo izquierda), así
            # que el riesgo de colisión lateral también puede venir del
            # lado derecho.
            self.get_logger().warn(
                f'PARA der={self.dist_der_raw:.3f}m — calibrando', throttle_duration_sec=0.4)
            self._calibrar_tras_stop()

        # ── CRUCERO ───────────────────────────────────────────────────────
        if self.estado == CRUCERO:
            ahora = self.get_clock().now().nanoseconds * 1e-9
            t_post = ahora - self.t_ultimo_giro
            cooldown_ok = t_post >= self.t_cooldown

            if cooldown_ok and self.dist_frente <= self.d_giro:
                # Girar hacia el lado con MÁS espacio disponible en este
                # instante (no siempre izquierda). Sin referencia de ningún
                # lado, izquierda por defecto.
                if math.isfinite(self.dist_izq) and math.isfinite(self.dist_der):
                    self.dir_giro = 1.0 if self.dist_izq >= self.dist_der else -1.0
                elif math.isfinite(self.dist_der):
                    self.dir_giro = -1.0
                else:
                    self.dir_giro = 1.0
                # Magnitud del giro: se calcula UNA vez aquí, con las
                # lecturas de este instante, y queda fija durante todo el
                # GIRO. Recalcularla en cada ciclo con dist_frente en vivo
                # la hacía errática: dist_frente cambia rápido y de forma
                # poco representativa mientras el robot rota, así que el
                # giro salía a veces corto, a veces excesivo.
                urgencia = max(0.0, self.d_giro - self.dist_frente) / max(self.d_giro, 1e-6)
                self.w_giro_efectivo = max(-self.max_w, min(self.max_w,
                    self.dir_giro * self.w_giro * (1.0 + self.k_urgencia_giro * urgencia)))
                self._cambiar(GIRO)
                self.get_logger().info(
                    f'GIRO dir={"izq" if self.dir_giro > 0 else "der"}'
                    f'  izq={self.dist_izq:.2f}  der={self.dist_der:.2f}'
                    f'  w_giro={self.w_giro_efectivo:.2f}')
                self._pub_dbg(0, 0, 0, 0)
                return

            recuperando = t_post < self.t_recuperacion
            w_front = 0.0
            w_der   = 0.0

            if recuperando:
                # Solo tracking pared derecha — permite alinearse tras RODEO
                if math.isfinite(self.dist_der):
                    w_der = self._w_der_pd()
            elif self.dist_frente < self.d_front_ini:
                # Evasión frontal pura — sin competencia con w_der
                w_front = self.Kfront * (self.d_front_ini - self.dist_frente)
            else:
                # Tracking normal pared derecha
                if math.isfinite(self.dist_der):
                    w_der = self._w_der_pd()

            # Repulsión izquierda (siempre activa)
            w_izq = 0.0
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_izq_min:
                w_izq = -self.Kizq * (self.d_izq_min - self.dist_izq)

            w = max(-self.max_w, min(self.max_w, w_front + w_der + w_izq))
            self._pub_dbg(w_front, w_der, w_izq, w)
            self._pub(self.v_cruise, w)

        # ── GIRO ──────────────────────────────────────────────────────────
        elif self.estado == GIRO:
            # d_lado: distancia del lado HACIA EL QUE SE GIRA (criterio de
            # salida por acercamiento excesivo a esa pared).
            d_lado = self.dist_izq if self.dir_giro > 0 else self.dist_der

            if self._t_estado() > self.t_giro_max:
                self._cambiar(RODEO)
                return
            if math.isfinite(d_lado) and d_lado < self.d_lado_salida_giro:
                self.get_logger().warn(
                    f'GIRO→RODEO lado={"izq" if self.dir_giro > 0 else "der"}={d_lado:.2f}m')
                self._cambiar(RODEO)
                return
            if self._t_estado() >= self.t_giro_min and self.dist_frente > self.d_giro:
                self._cambiar(RODEO)
                return

            # Magnitud congelada al entrar a GIRO (ver comentario en CRUCERO).
            # v_maniobra (más lenta que crucero): menos distancia recorrida
            # "a ciegas" por ciclo si el rumbo queda torcido tras el giro.
            self._pub_dbg(self.w_giro_efectivo, 0, 0, self.w_giro_efectivo)
            self._pub(self.v_maniobra, self.w_giro_efectivo)

        # ── RODEO: avance recto para separarse del obstáculo ──────────────
        elif self.estado == RODEO:
            t = self._t_estado()
            if t >= self.t_rodeo_max:
                self._cambiar(CRUCERO)
                return
            # Salida por sensor: recién evaluamos si el frente ya despejó
            # después de un mínimo, para no cortar el rodeo por un rebote
            # transitorio justo al salir de GIRO.
            if t >= self.t_rodeo_min and self.dist_frente > self.d_front_ini:
                self._cambiar(CRUCERO)
                return
            self._pub_dbg(0, 0, 0, 0)
            self._pub(self.v_maniobra, 0.0)   # w=0: absolutamente recto, más lento que crucero

    # ── Debug JSON para lidar_viz.py ─────────────────────────────────────────
    def _publicar_debug(self):
        segs = []
        for cl in self.clusters:
            for s in cl['segs']:
                # color por LÍNEA, no por cluster: cada segmento se pinta
                # naranja (caja) o azul (pared) según su propio largo.
                clase_seg = pc.CAJA if s['lon'] <= self.lado_caja_linea else pc.PARED
                segs.append({'x1': round(s['p1'][0], 3), 'y1': round(s['p1'][1], 3),
                             'x2': round(s['p2'][0], 3), 'y2': round(s['p2'][1], 3),
                             'lon': round(s['lon'], 3), 'clase': clase_seg})
        pared_der = None
        if math.isfinite(self.dist_der):
            pared_der = {'d': round(self.dist_der, 3), 'alpha_deg': 0.0,
                         'tipo': 'RANSAC_DER', 'd_front': None, 'd_rear': None}
        data = {
            'estado': self.estado, 'fase': 0,
            'accion': self.accion,
            'd_frente': None if not math.isfinite(self.dist_frente)
                        else round(self.dist_frente, 3),
            'd_atras': None if not math.isfinite(self.dist_atras)
                       else round(self.dist_atras, 3),
            'clase_frente': self.clase_frente,
            'pared_der': pared_der,
            'd_izq': None if not math.isfinite(self.dist_izq_raw)
                     else round(self.dist_izq_raw, 3),
            'd_der': None if not math.isfinite(self.dist_der_raw)
                     else round(self.dist_der_raw, 3),
            'pose': None if self.pose is None else [round(v, 3) for v in self.pose],
            'trail': [[round(x, 3), round(y, 3)] for x, y in self.trail[-400:]],
            'cajas_vivas': [[round(x, 3), round(y, 3)]
                            for x, y in self.cajas_vivas],
            'cajas_fijas': [],
            'segs': segs,
        }
        m = String()
        m.data = json.dumps(data)
        self.pub_debug.publish(m)


def _esperar_inicio(logger, segundos: float):
    """Cuenta regresiva antes de arrancar (tiempo para acomodar el robot en
    la pista). El nodo ya está creado (tópicos advertidos) pero no se
    procesa ningún callback hasta rclpy.spin(), así que nada se mueve
    mientras tanto. Sin ENTER: funciona igual con ros2 run o ros2 launch."""
    restante = segundos
    while restante > 0:
        logger.info(f'Arrancando en {restante:.0f}s...')
        time.sleep(1.0)
        restante -= 1.0
    logger.info('Arrancando!')


def main(args=None):
    rclpy.init(args=args)
    nodo = Guardian()
    _esperar_inicio(nodo.get_logger(), nodo.t_espera_inicio)
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
