#!/usr/bin/env python3
"""
behavior_fsm.py — Guardian v4 (RC3 + RETROCESO + heading): motor de
movimiento + detección de caja/pared por tamaño para el panel de
diagnóstico (lidar_viz.py).

Estados: CRUCERO → GIRO → RODEO → CRUCERO
         (cualquier estado) → RETROCESO → CRUCERO   [si hay choque/STOP,
         o si GIRO no resuelve a tiempo — espacio muy chico]

El motor de movimiento (control, STOP de seguridad, GIRO/RODEO/RETROCESO)
viene de RC3 (2) y RC3 (3): gira hacia el lado con MÁS espacio disponible
en cada momento (no siempre hacia la izquierda), RODEO avanza recto hasta
que el frente se despeja, y RETROCESO se aleja del choque (o de un GIRO
que no encontró salida) en vez de quedarse congelado o seguir a ciegas.

dist_frente y dist_izq_raw/dist_der_raw (mínimo crudo, sin filtrar) se
miden localmente sobre /scan: son la base del STOP de seguridad y deben
ver CUALQUIER objeto cercano. dist_izq / dist_der llegan del nodo
wall_follower (RANSAC sobre las paredes laterales, vía /dist_izq y
/dist_der) y se usan para la elección de lado en GIRO/RETROCESO y como
respaldo del tracking de crucero — RANSAC descarta a propósito los
objetos que no son pared (p.ej. una caja pegada al costado) como
outliers, por lo que NO sirven para detectar un choque lateral inminente.

Aparte del motor de movimiento, este nodo corre en paralelo la
clasificación caja/pared de percepcion.py (Split-and-Merge por tamaño de
línea: ≤ lado_caja_linea → caja, mayor → pared) para poblar
/guardian/debug (clase_frente, segmentos coloreados, cajas vivas, trail)
que consume lidar_viz.py. Esa clasificación NO decide qué es caja/pared
para el movimiento en sí — pero el segmento PARED del borde exterior que
produce SÍ alimenta _w_der_pd() con distancia + heading (ángulo del
segmento respecto al avance), en vez de solo la distancia de RANSAC;
si no hay segmento clasificado disponible, cae a RANSAC sin heading.

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
from std_msgs.msg import Float32, Int32, String

from behavior_fsm import percepcion as pc


CRUCERO   = 'CRUCERO'
GIRO      = 'GIRO'
RODEO     = 'RODEO'
RETROCESO = 'RETROCESO'


class Guardian(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros de movimiento (RC3) ───────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  30.0)
        self.declare_parameter('t_espera_inicio',     10.0)  # s  cuenta regresiva tras ENTER

        self.declare_parameter('d_stop_front',   0.14)
        self.declare_parameter('d_stop_lateral', 0.06)
        self.declare_parameter('d_giro',         0.28)
        self.declare_parameter('d_front_inicio', 0.40)

        self.declare_parameter('target_der', 0.17)   # robot consistente a 17-20cm
        self.declare_parameter('Kder',        2.6)
        self.declare_parameter('Kd_der',      0.3)
        self.declare_parameter('d_izq_min',  0.15)
        self.declare_parameter('Kizq',        4.0)
        self.declare_parameter('Kfront',      2.0)

        self.declare_parameter('vel_crucero',      0.10)
        self.declare_parameter('vel_maniobra',     0.035)  # reducida: menos distancia recorrida en el giro
        self.declare_parameter('vel_giro_gradual', 0.40)
        self.declare_parameter('max_w',            0.60)

        self.declare_parameter('t_giro_min',        0.7)
        self.declare_parameter('t_giro_max',        2.0)  # baja de 4.0: acota el giro
                                                            # máximo en un solo intento
                                                            # (~70° a max_w) — si no
                                                            # resuelve, retrocede en vez
                                                            # de seguir girando/avanzando
        self.declare_parameter('d_lado_salida_giro', 0.20)
        self.declare_parameter('k_urgencia_giro',   1.2)
        self.declare_parameter('d_lado_max_creible', 1.0)  # m  por encima de esto,
                                                            # una lectura RANSAC no es
                                                            # "espacio libre" sino "sin
                                                            # pared real que ajustar"
        self.declare_parameter('t_rodeo_min',       0.4)
        self.declare_parameter('t_rodeo_max',       1.0)
        self.declare_parameter('t_cooldown',        2.0)
        self.declare_parameter('t_recuperacion',    1.5)

        # Retroceso tras un choque (STOP absoluto) — línea recta, sin girar
        self.declare_parameter('dist_retroceso', 0.15)  # m  retrocede esto tras un choque
        self.declare_parameter('vel_retroceso',  0.08)  # m/s magnitud (t_retroceso = dist/vel)

        # Tracking de pared derecha con heading (además de distancia): usa el
        # segmento PARED ya clasificado por percepcion.py (borde exterior del
        # jirón) en vez de solo la distancia de RANSAC, sumando el término de
        # ángulo que evita el zigzag del PID de una sola variable.
        self.declare_parameter('K_alpha', 1.0)            # ganancia del heading
        self.declare_parameter('min_long_pared_pd', 0.30)  # m mínimo del segmento de referencia
        self.declare_parameter('cos_lateral_min_pd', 0.55)  # |dx|/L mínimo ("va a lo largo de x")

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
        self.declare_parameter('offset_lados', 0.08)
        self.declare_parameter('topic_odom', '/odom_raw')

        # ── Mapa fijo de pared (marco odom, solo panel) ────────────────────
        # Nube de PUNTOS fundidos por grilla (no líneas): unir segmentos
        # completos entre pasadas es frágil (casi nunca comparten extremos
        # exactos), unir cada punto a la celda más cercana es simple y
        # converge solo con el uso.
        self.declare_parameter('mapa_rango_max', 0.55)  # m  no mapear detecciones
                                                          # lejanas: el error de yaw
                                                          # de la odometría se amplifica
                                                          # con el rango (a 1.5m, 20° de
                                                          # error de yaw ya desplaza el
                                                          # punto 0.5m — de ahí salía el
                                                          # mapa "en espiral")
        self.declare_parameter('mapa_celda', 0.04)      # m  lado de la celda de fusión
        self.declare_parameter('mapa_max_puntos', 6000)  # tope de celdas distintas

        # ── Censo lado derecho (marco odom, solo panel) ─────────────────────
        self.declare_parameter('censo_dist_dup', 0.30)     # m  radio de deduplicado
        self.declare_parameter('censo_confirmaciones', 3)  # hits para confirmar
        self.declare_parameter('censo_max', 8)             # tope de cajas censadas

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
        self.off_l = g('offset_lados')
        self.mapa_rango_max = g('mapa_rango_max')
        self.mapa_celda     = g('mapa_celda')
        self.mapa_max_puntos = int(g('mapa_max_puntos'))
        self.censo_dist_dup = g('censo_dist_dup')
        self.censo_confirmaciones = int(g('censo_confirmaciones'))
        self.censo_max      = int(g('censo_max'))

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
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self.dist_izq_raw = float('inf')
        self.dist_der_raw = float('inf')

        # ── Percepción / clasificación (solo para el panel) ───────────────
        self.clusters = []
        self.clase_frente = None
        self.pared_der_seg = None  # {'d','alpha','lon'} del borde exterior, o None
        self.accion = 'AVANZANDO'

        # ── Odometría / censo (para el panel derecho) ─────────────────────
        self.pose = None
        self.trail = []
        self.cajas_vivas = []
        self.cajas_fijas = []       # censo confirmado, solo lado derecho (el que se sigue)
        self.cajas_pendientes = []  # candidatas a confirmar (hits/miss), ver _actualizar_censo_derecha
        self.mapa_pared = {}    # {(celda_x,celda_y): {'x','y','n'}} nube de puntos en marco odom

        # ── Métricas para metrics_logger.py (metricas_lidar.csv) ───────────
        self.n_colisiones     = 0     # STOP por proximidad real (no timeouts de GIRO)
        self.n_rodeo_intentos = 0     # cada CRUCERO→GIRO es un intento de evasión
        self.n_rodeo_exitosos = 0     # GIRO→RODEO→CRUCERO sin choque de por medio
        self._rodeo_actual_ok = True  # se apaga si el intento en curso choca

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

        # Para metrics_logger.py: distancia de frenado frente a una caja,
        # y contadores de colisiones/rodeos exitosos de esta corrida.
        self.pub_parada     = self.create_publisher(Float32, '/parada_dist', 10)
        self.pub_colisiones = self.create_publisher(Int32,   '/metrics/colisiones', 10)
        self.pub_rodeo      = self.create_publisher(String,  '/metrics/rodeo_exitoso', 10)

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
        self.d_lado_max_creible = self.get_parameter('d_lado_max_creible').value
        self.t_rodeo_min       = self.get_parameter('t_rodeo_min').value
        self.t_rodeo_max       = self.get_parameter('t_rodeo_max').value
        self.t_cooldown        = self.get_parameter('t_cooldown').value
        self.t_recuperacion    = self.get_parameter('t_recuperacion').value
        self.dist_retroceso    = self.get_parameter('dist_retroceso').value
        self.vel_retroceso     = self.get_parameter('vel_retroceso').value
        self.t_retroceso       = self.dist_retroceso / max(self.vel_retroceso, 1e-6)
        self.K_alpha           = self.get_parameter('K_alpha').value
        self.min_long_pared_pd = self.get_parameter('min_long_pared_pd').value
        self.cos_lat_pd        = self.get_parameter('cos_lateral_min_pd').value

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
        vistas_der = []
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
            if cy < -0.05:      # solo el lado que el robot sigue (derecha)
                vistas_der.append(odom)
        self.cajas_vivas = vivas[-6:]
        self._actualizar_censo_derecha(vistas_der)

    def _actualizar_censo_derecha(self, vistas):
        """Censo confirmado de cajas SOLO del lado derecho (el que el
        robot sigue pegado a la pared) — cada caja real debe aparecer ahí
        al pasarla, así que filtrar por lado descarta clusters CAJA
        espurios que se vean de refilón hacia la izquierda (isla). Misma
        lógica de confirmación por hits/miss que box_detector.py (Parte
        A), pero local a este panel — no reemplaza el censo oficial."""
        for p in self.cajas_pendientes:
            p['miss'] += 1
        for ox, oy in vistas:
            if any(math.hypot(ox - fx, oy - fy) < self.censo_dist_dup
                   for fx, fy in self.cajas_fijas):
                continue
            mejor = None
            for p in self.cajas_pendientes:
                d = math.hypot(ox - p['x'], oy - p['y'])
                if d < self.censo_dist_dup and (mejor is None or d < mejor[0]):
                    mejor = (d, p)
            if mejor is None:
                self.cajas_pendientes.append({'x': ox, 'y': oy, 'hits': 1, 'miss': 0})
                continue
            p = mejor[1]
            h = p['hits']
            p['x'] = (p['x'] * h + ox) / (h + 1)
            p['y'] = (p['y'] * h + oy) / (h + 1)
            p['hits'] = h + 1
            p['miss'] = 0
            if p['hits'] >= self.censo_confirmaciones and len(self.cajas_fijas) < self.censo_max:
                self.cajas_fijas.append((p['x'], p['y']))
        self.cajas_pendientes = [p for p in self.cajas_pendientes
                                 if p['miss'] <= 8 and p['hits'] < self.censo_confirmaciones]

    def _actualizar_mapa_pared(self, clusters):
        """Mapa FIJO de la pista en marco odom: nube de PUNTOS (no líneas).
        Cada punto crudo de un cluster PARED se transforma a odom con la
        pose actual y se funde con lo ya mapeado por cercanía de grilla —
        unir SEGMENTOS completos entre pasadas es frágil (casi nunca
        comparten extremos exactos: el mismo tramo de pared se ve con
        largos y cortes distintos según el ángulo y qué tanto lo tapó una
        caja), unir PUNTO a PUNTO es simple y converge solo con el uso.

        Filtros para que el punto sea confiable:
          · NO se mapea durante GIRO/RETROCESO, ni en el primer tramo de
            CRUCERO/RODEO tras uno (self.t_recuperacion) — la orientación
            recién salida de un giro es la menos confiable, y un error de
            yaw se amplifica con la distancia al transformar a odom.
          · Se descarta cualquier punto a más de mapa_rango_max — mismo
            motivo, el error crece con el rango.
          · Cada punto cae en una celda de mapa_celda metros; si la celda
            ya tiene un punto, se promedia (running average) en vez de
            agregar uno nuevo — así el mapa no crece sin límite ni se ve
            "peludo" de puntos casi duplicados.
        """
        if self.pose is None or self.estado in (GIRO, RETROCESO):
            return
        ahora = self.get_clock().now().nanoseconds * 1e-9
        if ahora - self.t_ultimo_giro < self.t_recuperacion:
            return
        celda = self.mapa_celda
        for cl in clusters:
            if cl['clase'] != pc.PARED:
                continue
            for (px, py) in cl['pts']:
                if math.hypot(px, py) > self.mapa_rango_max:
                    continue
                odom = self._a_odom(px, py)
                if odom is None:
                    continue
                ox, oy = odom
                clave = (round(ox / celda), round(oy / celda))
                entrada = self.mapa_pared.get(clave)
                if entrada is None:
                    if len(self.mapa_pared) < self.mapa_max_puntos:
                        self.mapa_pared[clave] = {'x': ox, 'y': oy, 'n': 1}
                    continue
                n = entrada['n']
                entrada['x'] = (entrada['x'] * n + ox) / (n + 1)
                entrada['y'] = (entrada['y'] * n + oy) / (n + 1)
                entrada['n'] = n + 1

    def cb_scan(self, msg: LaserScan):
        # Frente + mínimo crudo lateral: se miden aquí mismo, sin depender de
        # otro nodo ni de RANSAC, porque son la base del STOP de seguridad y
        # necesitan ver CUALQUIER objeto cercano (pared o caja), no solo la
        # pared "limpia" que RANSAC reporta para el tracking de crucero.
        d_f = float('inf')
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
            elif af > 0:
                d_l_raw = min(d_l_raw, r)
            else:
                d_r_raw = min(d_r_raw, r)
        self.dist_frente   = d_f
        self.dist_izq_raw  = d_l_raw
        self.dist_der_raw  = d_r_raw

        # Clasificación caja/pared (percepcion.py) — la caja/pared la decide
        # SOLO esto (igual que antes, para el panel); pero el segmento PARED
        # del borde exterior que sale de aquí también alimenta el tracking
        # de crucero (_w_der_pd) con distancia + heading, no solo distancia
        # RANSAC como antes.
        pts = pc.filtrar_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max,
                              self.front_rad, self.rango_max_clasif, self.atras_rad)
        self.clusters = pc.analizar_scan(pts, self.salto_dist, self.salto_idx,
                                         self.umbral_split, self.min_puntos,
                                         self.lado_caja)
        _, clase_f, _, _, _, _ = pc.frente_y_lados(
            pts, self.clusters, self.off_f, self.off_l)
        self.clase_frente = clase_f
        self.pared_der_seg = pc.pared_derecha(self.clusters, self.min_long_pared_pd,
                                              self.cos_lat_pd)
        self._actualizar_cajas_desde_scan(self.clusters)
        self._actualizar_mapa_pared(self.clusters)

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
        if self.estado == RETROCESO:
            self.accion = 'RETROCEDIENDO'
        elif self.estado == GIRO:
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
        """PD de tracking a la pared derecha, con heading además de
        distancia: usa el segmento PARED del borde exterior ya clasificado
        (percepcion.py) — su 'alpha' (ángulo respecto al avance) corrige la
        orientación además de la distancia, que es lo que evita el zigzag
        de un PD de una sola variable. Si no hay segmento clasificado
        disponible (p.ej. muy cerca de una esquina), cae a la distancia
        RANSAC de wall_follower sin corrección de heading (alpha=0), igual
        que el comportamiento anterior."""
        if self.pared_der_seg is not None:
            d_der = self.pared_der_seg['d']
            alpha = self.pared_der_seg['alpha']
        else:
            d_der = self.dist_der
            alpha = 0.0
        error = d_der - self.target_der
        now   = self.get_clock().now()
        dt    = max((now - self._t_der_prev).nanoseconds * 1e-9, 0.01)
        d_err = (error - self._err_der_prev) / dt
        self._err_der_prev = error
        self._t_der_prev   = now
        # lejos → girar derecha (w<0); alpha≠0 → realinear paralelo
        return -(self.Kder * error + self.Kd_der * d_err) + self.K_alpha * alpha

    def _cambiar(self, nuevo: str):
        self.get_logger().info(
            f'{self.estado}→{nuevo}  '
            f'f={self.dist_frente:.2f}  l={self.dist_izq:.2f}  r={self.dist_der:.2f}'
            f'  clase_frente={self.clase_frente}')
        # Cooldown de GIRO: SOLO tras un GIRO/RODEO normal — evita reactivar
        # GIRO apenas se termina de rodear, por proximidad residual del giro
        # mismo. NO se aplica tras RETROCESO: ahí el obstáculo que causó el
        # choque puede seguir justo enfrente, y CRUCERO necesita poder
        # re-evaluar GIRO de inmediato. Aplicarlo también tras RETROCESO
        # dejaba al robot ciego a GIRO 1.5-2s, arrastrándose hacia el mismo
        # obstáculo y disparando CHOQUE→RETROCESO otra vez sin girar nunca
        # (visto en pruebas reales: ~10 ciclos seguidos sin avanzar).
        if self.estado in (GIRO, RODEO) and nuevo == CRUCERO:
            self.t_ultimo_giro = self.get_clock().now().nanoseconds * 1e-9
        # Resetea la derivada del PD al volver a CRUCERO desde cualquier
        # maniobra ciega (GIRO/RODEO/RETROCESO): si no, el primer tick
        # calcula d_err sobre un salto acumulado durante la maniobra
        # (spike falso).
        if self.estado in (GIRO, RODEO, RETROCESO) and nuevo == CRUCERO:
            if math.isfinite(self.dist_der):
                self._err_der_prev = self.dist_der - self.target_der
            self._t_der_prev = self.get_clock().now()
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def _iniciar_retroceso(self, motivo: str):
        """Retrocede en línea RECTA (sin girar) una distancia fija. Girar
        mientras se retrocede (versión anterior) podía barrer el costado
        del robot hacia otro obstáculo cercano en vez de alejarlo, y
        terminaba en un vaivén choca→retrocede→choca sin escapar nunca
        (visto en pruebas reales, varias veces seguidas). En línea recta
        el retroceso solo aumenta la distancia al punto de choque, sin
        arriesgar un golpe lateral nuevo por el giro.

        También alimenta metrics_logger.py: cualquier retroceso echa a
        perder el intento de rodeo en curso (self._rodeo_actual_ok), y un
        CHOQUE (no un GIRO sin resolver, que no es un choque real) cuenta
        como colisión y, si fue de frente contra una CAJA, reporta la
        distancia de frenado (/parada_dist)."""
        self._rodeo_actual_ok = False
        if motivo.startswith('CHOQUE'):
            self.n_colisiones += 1
            self.pub_colisiones.publish(Int32(data=self.n_colisiones))
            if motivo.startswith('CHOQUE frente=') and self.clase_frente == pc.CAJA:
                self.pub_parada.publish(Float32(data=float(self.dist_frente)))
        self.get_logger().warn(
            f'{motivo} → retrocede {self.dist_retroceso:.2f}m recto',
            throttle_duration_sec=0.4)
        self._cambiar(RETROCESO)
        self._pub_dbg(0, 0, 0, 0)
        self._pub(-self.vel_retroceso, 0.0)

    def _cerrar_rodeo(self):
        """Cierra el intento de rodeo actual para metrics_logger.py: si no
        hubo ningún choque de por medio (self._rodeo_actual_ok), cuenta
        como exitoso. Se llama justo antes de volver a CRUCERO desde
        RODEO limpio (nunca desde RETROCESO, que ya marcó el intento
        como fallido en _iniciar_retroceso)."""
        if self._rodeo_actual_ok:
            self.n_rodeo_exitosos += 1
        self.pub_rodeo.publish(String(data=f'{self.n_rodeo_exitosos}/{self.n_rodeo_intentos}'))

    def loop_control(self):

        # ── PRIORIDAD 1: STOP absoluto → retrocede recto ───────────────────
        # No se evalúa si ya estamos en RETROCESO: dist_frente sigue por
        # debajo de d_stop_front justo al entrar (es la causa del choque), y
        # si el STOP se re-disparara aquí el robot se quedaría parado para
        # siempre sin llegar nunca a ejecutar el retroceso.
        if self.estado != RETROCESO:
            choque = None
            if self.dist_frente < self.d_stop_front:
                choque = f'frente={self.dist_frente:.3f}m'
            # Usa el mínimo crudo (dist_izq_raw/dist_der_raw), NO el de
            # RANSAC: RANSAC descarta a propósito objetos que no son pared
            # (p.ej. una caja pegada al costado) como outliers, y ese es
            # justo el caso que este STOP tiene que detectar.
            elif math.isfinite(self.dist_izq_raw) and self.dist_izq_raw < self.d_stop_lat:
                choque = f'izq={self.dist_izq_raw:.3f}m'
            elif math.isfinite(self.dist_der_raw) and self.dist_der_raw < self.d_stop_lat:
                choque = f'der={self.dist_der_raw:.3f}m'

            if choque is not None:
                self._iniciar_retroceso(f'CHOQUE {choque}')
                return

        # ── RETROCESO: recto hacia atrás, sin girar, tiempo fijo ───────────
        if self.estado == RETROCESO:
            if self._t_estado() >= self.t_retroceso:
                self._cambiar(CRUCERO)
                return
            self._pub_dbg(0, 0, 0, 0)
            self._pub(-self.vel_retroceso, 0.0)
            return

        # ── CRUCERO ───────────────────────────────────────────────────────
        if self.estado == CRUCERO:
            ahora = self.get_clock().now().nanoseconds * 1e-9
            t_post = ahora - self.t_ultimo_giro
            cooldown_ok = t_post >= self.t_cooldown

            if cooldown_ok and self.dist_frente <= self.d_giro:
                # Girar hacia el lado con MÁS espacio CREÍBLE. Una lectura
                # RANSAC por encima de d_lado_max_creible no es "espacio
                # libre" — el corredor real es mucho más angosto que eso,
                # así que es RANSAC sin pared izquierda/derecha real que
                # ajustar (cayó en un punto lejano/ruido). Tratarla como
                # "espacio abierto" mandaba al robot a girar bien lejos de
                # su carril de referencia y no lograba volver (visto en
                # pruebas reales: izq=2.5-2.9m junto a una caja pegada al
                # frente-derecha). Por defecto, y si ninguna lectura es
                # creíble, se prefiere el lado DERECHO — es el que el
                # robot sigue durante todo el crucero.
                iz = self.dist_izq if (math.isfinite(self.dist_izq)
                                       and self.dist_izq <= self.d_lado_max_creible) else None
                de = self.dist_der if (math.isfinite(self.dist_der)
                                       and self.dist_der <= self.d_lado_max_creible) else None
                if iz is not None and de is not None:
                    self.dir_giro = 1.0 if iz >= de else -1.0
                elif iz is not None:
                    self.dir_giro = 1.0
                else:
                    self.dir_giro = -1.0
                # Magnitud del giro: se calcula UNA vez aquí, con las
                # lecturas de este instante, y queda fija durante todo el
                # GIRO. Recalcularla en cada ciclo con dist_frente en vivo
                # la hacía errática: dist_frente cambia rápido y de forma
                # poco representativa mientras el robot rota, así que el
                # giro salía a veces corto, a veces excesivo.
                urgencia = max(0.0, self.d_giro - self.dist_frente) / max(self.d_giro, 1e-6)
                self.w_giro_efectivo = max(-self.max_w, min(self.max_w,
                    self.dir_giro * self.w_giro * (1.0 + self.k_urgencia_giro * urgencia)))
                self.n_rodeo_intentos += 1   # nuevo intento de evasión, para metrics_logger
                self._rodeo_actual_ok = True
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
            # d_lado: distancia CRUDA (no RANSAC) del lado HACIA EL QUE SE
            # GIRA — criterio de salida por acercamiento excesivo a esa
            # pared. Con el valor de RANSAC este chequeo podía quedar
            # "ciego" (isfinite falla y se salta) justo cuando lo que se
            # acerca es la pared misma durante el giro, que es el caso que
            # tiene que frenar.
            d_lado = self.dist_izq_raw if self.dir_giro > 0 else self.dist_der_raw

            if self._t_estado() > self.t_giro_max:
                # No resolvió a tiempo: el espacio es muy chico para
                # completar el giro normal. Seguir rotando a ciegas hacia
                # RODEO (avance recto) es lo que puede terminar en un giro
                # casi completo en un lugar chico — mejor retroceder para
                # abrir espacio y reintentar, igual que ante un choque.
                self._iniciar_retroceso('GIRO sin resolver')
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
                self._cerrar_rodeo()
                self._cambiar(CRUCERO)
                return
            # Salida por sensor: recién evaluamos si el frente ya despejó
            # después de un mínimo, para no cortar el rodeo por un rebote
            # transitorio justo al salir de GIRO.
            if t >= self.t_rodeo_min and self.dist_frente > self.d_front_ini:
                self._cerrar_rodeo()
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
        if self.pared_der_seg is not None:
            pared_der = {'d': round(self.pared_der_seg['d'], 3),
                         'alpha_deg': round(math.degrees(self.pared_der_seg['alpha']), 1),
                         'tipo': 'PERCEPCION_DER', 'd_front': None, 'd_rear': None}
        elif math.isfinite(self.dist_der):
            pared_der = {'d': round(self.dist_der, 3), 'alpha_deg': 0.0,
                         'tipo': 'RANSAC_DER', 'd_front': None, 'd_rear': None}
        data = {
            'estado': self.estado, 'fase': 0,
            'accion': self.accion,
            'd_frente': None if not math.isfinite(self.dist_frente)
                        else round(self.dist_frente, 3),
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
            'cajas_fijas': [[round(x, 3), round(y, 3)] for x, y in self.cajas_fijas],
            'mapa_pared': [[round(e['x'], 3), round(e['y'], 3)]
                           for e in self.mapa_pared.values()],
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
