#!/usr/bin/env python3
"""
lidar_viz.py — Monitor visual en tiempo real del robot CapyTown.
VERSION: v6

Panel izquierdo : mapa LiDAR con sector frontal coloreado por estado.
  - cyan   : sector ±12° libre (sin caja)
  - naranja: caja 25 cm perpendicular detectada → se dibuja rectángulo en el mapa
Panel derecho   : DER (grande) + IZQ (grande) + historial velocidad.

Deteccion de caja: std_x < 4 cm (perpendicular) + 8 cm <= ancho <= 32 cm (25 cm ± margen)
"""

VIZ_VERSION = 'v25'

import math
import argparse
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
import numpy as np


# ── Paleta ────────────────────────────────────────────────────────────────────
BG          = '#0d1117'
PANEL       = '#161b22'
BORDER      = '#30363d'
C_FRONT_OK  = '#00bcd4'   # cyan: sector frontal sin caja
C_FRONT_BOX = '#f39c12'   # naranja: caja perpendicular detectada
C_LEFT      = '#4f8ef7'   # azul: sector izquierdo
C_RIGHT     = '#2ecc71'   # verde: sector derecho
C_OTHER     = '#3a4a5a'   # gris: resto
C_BOX       = '#f39c12'   # naranja: cajas censadas en odom
C_RODEO     = '#b07fff'   # purpura: estado RODEO
C_TRAJ      = '#8b949e'
C_ARROW     = '#f5c518'
C_TEXT      = '#e6edf3'
C_DIM       = '#8b949e'
C_WARN      = '#f39c12'
C_ALERT     = '#e94560'
C_OK        = '#2ecc71'

# ── Constantes ────────────────────────────────────────────────────────────────
FRONT_GIRO_HALF = math.radians(12.0)   # ±12° dispara GIRO en la FSM
SIDE_LO    = math.radians(60.0)
SIDE_HI    = math.radians(120.0)

TARGET_DER   = 0.08   # m  objetivo pared derecha
TOL_OK       = 0.02   # m  tolerancia OK
MAX_GAUGE    = 0.70   # m  maximo del gauge

DIST_IZQ_MIN  = 0.15  # m  zona de repulsion (= dist_izq_min en FSM)
DIST_IZQ_WARN = 0.25  # m  inicio de advertencia

# Deteccion de caja (~25 cm): perpendicular (std_x) + ancho lateral
DETECT_HALF   = math.radians(20.0)  # sector de busqueda para caja
DETECT_MAX_R  = 0.45   # m  rango maximo de busqueda
PERP_STD_MAX  = 0.04   # m  max std de profundidad (x). perpendicular → std_x≈0
BOX_W_MIN     = 0.08   # m  ancho minimo de caja
BOX_W_MAX     = 0.23   # m  ancho maximo de caja (20 cm + 3 cm margen de medicion)
MIN_FRONT_PTS = 5      # minimo de puntos para considerar deteccion valida

MAX_TRAJ    = 400
MAX_VEL_T   = 10.0


# ── Nodo ROS ──────────────────────────────────────────────────────────────────
class LidarViz(Node):
    def __init__(self, front_deg: float):
        super().__init__('lidar_viz')
        self.front_rad = math.radians(front_deg)

        self.scan_segs     = []
        self.scan_seg_cols = []
        self.d_front    = float('inf')
        self.d_left     = float('inf')
        self.d_right    = float('inf')
        self.range_min  = 0.0
        self.range_max  = 8.0

        self.box_frente      = False
        self.box_frente_dist = float('inf')
        self.box_cx          = 0.0   # profundidad (x robot) del centro de la caja
        self.box_cy          = 0.0   # lateral (y robot) del centro de la caja
        self.box_w           = 0.0   # ancho detectado de la caja

        self.fsm_state  = 'CRUCERO'

        self.vel_lin    = 0.0
        self.vel_ang    = 0.0
        self.lat_corr   = 0.0
        self.cajas_odom = []

        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.robot_yaw  = 0.0
        self.traj_odom  = deque(maxlen=MAX_TRAJ)

        self.vel_times  = deque()
        self.vel_vals   = deque()
        self._t0        = time.time()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan',               self._cb_scan,   qos)
        self.create_subscription(Odometry,  '/odom_raw',           self._cb_odom,   qos)
        self.create_subscription(Twist,     '/cmd_vel',            self._cb_cmd,    10)
        self.create_subscription(Float32,   '/lateral_correction', self._cb_lat,    10)
        self.create_subscription(PoseArray, '/cajas_avistadas',    self._cb_cajas,  10)
        self.create_subscription(String,    '/fsm_state',          self._cb_estado, 10)

    def _sensor_to_display(self, theta, r):
        phi = self.front_rad - theta
        return r * math.sin(phi), r * math.cos(phi)

    def _odom_to_display(self, ox, oy):
        dx, dy = ox - self.robot_x, oy - self.robot_y
        c = math.cos(-self.robot_yaw); s = math.sin(-self.robot_yaw)
        xb = c * dx - s * dy
        yb = s * dx + c * dy
        return -yb, xb

    def _cb_scan(self, msg: LaserScan):
        segs      = []
        seg_cols  = []
        d_f = d_l = d_r = float('inf')
        front_rf  = []
        prev_xd = prev_yd = prev_r = None

        for i in range(0, len(msg.ranges), 4):
            r = msg.ranges[i]
            if not math.isfinite(r) or r == 0.0:
                prev_xd = prev_yd = prev_r = None
                continue
            if r < msg.range_min or r > msg.range_max:
                prev_xd = prev_yd = prev_r = None
                continue

            theta  = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(theta - self.front_rad),
                                math.cos(theta - self.front_rad))
            abs_af = abs(af)

            # Coloreado y medicion de distancias por sector
            if abs_af <= FRONT_GIRO_HALF:
                color = '__front__'   # placeholder: naranja o cyan segun deteccion
                d_f = min(d_f, r)
            elif SIDE_LO <= abs_af <= SIDE_HI:
                if af > 0:
                    color = C_LEFT;  d_l = min(d_l, r)
                else:
                    color = C_RIGHT; d_r = min(d_r, r)
            else:
                color = C_OTHER

            # Colectar puntos cercanos en sector de deteccion (independiente del color)
            if abs_af <= DETECT_HALF and r <= DETECT_MAX_R:
                front_rf.append((r * math.cos(af), r * math.sin(af)))

            xd, yd = self._sensor_to_display(theta, r)
            if (prev_xd is not None and prev_r is not None
                    and abs(r - prev_r) < 0.20
                    and math.hypot(xd - prev_xd, yd - prev_yd) < 0.30):
                segs.append([(prev_xd, prev_yd), (xd, yd)])
                seg_cols.append(color)

            prev_xd, prev_yd, prev_r = xd, yd, r

        self.d_front = d_f
        self.d_left  = d_l
        self.d_right = d_r

        # ── Deteccion de caja (~25 cm perpendicular) ─────────────────────
        # Doble chequeo:
        #   1. std_x < PERP_STD_MAX  → todos los puntos a la misma profundidad
        #      (superficie perpendicular al robot). Pared diagonal → std_x grande.
        #   2. BOX_W_MIN <= y_spread <= BOX_W_MAX → ancho compatible con caja 25 cm.
        #      Filtra paredes largas (> 32 cm en el cono) y puntos sueltos (< 8 cm).
        self.box_frente      = False
        self.box_frente_dist = float('inf')
        self.box_cx = self.box_cy = self.box_w = 0.0
        if len(front_rf) >= MIN_FRONT_PTS:
            xs    = [p[0] for p in front_rf]
            ys    = [p[1] for p in front_rf]
            n     = len(xs)
            mx    = sum(xs) / n
            std_x = math.sqrt(sum((x - mx) ** 2 for x in xs) / n)
            y_spread = max(ys) - min(ys)
            if std_x < PERP_STD_MAX and BOX_W_MIN <= y_spread <= BOX_W_MAX:
                self.box_frente      = True
                self.box_frente_dist = mx
                self.box_cx          = mx
                self.box_cy          = (max(ys) + min(ys)) / 2.0
                self.box_w           = y_spread

        # Asignar color definitivo al sector frontal segun si hay caja o no
        front_color = C_FRONT_BOX if self.box_frente else C_FRONT_OK
        self.scan_segs     = segs
        self.scan_seg_cols = [front_color if c == '__front__' else c
                              for c in seg_cols]

    def _cb_cmd(self, msg: Twist):
        self.vel_lin = msg.linear.x
        self.vel_ang = msg.angular.z
        t = time.time() - self._t0
        self.vel_times.append(t)
        self.vel_vals.append(msg.linear.x)
        while self.vel_times and (t - self.vel_times[0]) > MAX_VEL_T:
            self.vel_times.popleft()
            self.vel_vals.popleft()

    def _cb_lat(self, msg: Float32):
        self.lat_corr = msg.data

    def _cb_estado(self, msg: String):
        self.fsm_state = msg.data

    def _cb_cajas(self, msg: PoseArray):
        self.cajas_odom = [(p.position.x, p.position.y) for p in msg.poses]

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = p.x
        self.robot_y = p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        self.robot_yaw = math.atan2(siny, cosy)
        self.traj_odom.append((p.x, p.y))


# ── Helpers para construir un panel de distancia ──────────────────────────────
def _build_dist_panel(ax, title, title_color):
    """Panel generico con numero grande, barra y status. Devuelve dict de artistas."""
    ax.axis('off')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_title(title, color=title_color, fontsize=12, pad=6)

    # Numero grande en el centro
    lbl_val = ax.text(0.5, 0.60, '---', ha='center', va='center',
                      color=C_TEXT, fontsize=36, fontweight='bold')
    # Status (OK / CERCA / ALERTA etc.)
    lbl_status = ax.text(0.5, 0.22, '', ha='center', va='center',
                         color=C_DIM, fontsize=11)
    return {'lbl_val': lbl_val, 'lbl_status': lbl_status}


# ── Figura ────────────────────────────────────────────────────────────────────
def build_figure():
    fig = plt.figure(figsize=(12, 7), facecolor=BG)
    # 3 filas en la columna derecha: DER (grande), IZQ (grande), vel (pequena)
    gs = gridspec.GridSpec(3, 2,
                           width_ratios=[2.2, 1],
                           height_ratios=[2.2, 2.2, 1.0],
                           hspace=0.45, wspace=0.28,
                           left=0.05, right=0.97,
                           top=0.94, bottom=0.07)

    ax_lidar = fig.add_subplot(gs[:, 0])
    ax_der   = fig.add_subplot(gs[0, 1])
    ax_izq   = fig.add_subplot(gs[1, 1])
    ax_vel   = fig.add_subplot(gs[2, 1])

    for ax in [ax_lidar, ax_der, ax_izq, ax_vel]:
        ax.set_facecolor(PANEL)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)
        ax.tick_params(colors=C_DIM, labelsize=8)

    # ── LiDAR ─────────────────────────────────────────────────────────────
    ax_lidar.set_xlim(-1.5, 1.5)
    ax_lidar.set_ylim(-1.5, 1.5)
    ax_lidar.set_aspect('equal')
    ax_lidar.grid(True, color=BORDER, lw=0.5, ls='--')
    ax_lidar.set_title(f'LiDAR — Vista en tiempo real  [{VIZ_VERSION}]', color=C_TEXT, fontsize=12, pad=8)
    ax_lidar.set_xlabel('← IZQ   DER →', color=C_DIM, fontsize=8)
    ax_lidar.set_ylabel('↑ FRENTE', color=C_DIM, fontsize=8)
    for d, label in [(0.3, '30cm'), (0.5, '50cm'), (1.0, '1m'), (1.5, '1.5m')]:
        ax_lidar.add_patch(plt.Circle((0, 0), d, color=BORDER, fill=False, lw=0.7, ls=':'))
        ax_lidar.text(0.01, d + 0.02, label, color=BORDER, fontsize=7)
    ax_lidar.scatter([0], [0], s=80, c='white', zorder=6)
    ax_lidar.annotate('', xy=(0, 0.25), xytext=(0, 0),
                      arrowprops=dict(arrowstyle='->', color=C_ARROW, lw=2.5), zorder=7)
    lc = LineCollection([], linewidths=3.0, zorder=4)
    ax_lidar.add_collection(lc)
    traj_line, = ax_lidar.plot([], [], color=C_TRAJ, lw=1.2, alpha=0.6, zorder=3)
    box_patches = []
    range_circ = plt.Circle((0, 0), 1.5, color='#223344', fill=False, lw=1, ls='--', zorder=2)
    ax_lidar.add_patch(range_circ)

    # Alerta frontal — texto abajo del lidar (solo cuando hay caja)
    alert_txt = ax_lidar.text(0, -1.42, '', color=C_ALERT, fontsize=12,
                              ha='center', va='bottom', fontweight='bold', zorder=8)
    # Texto de caja en parte superior del lidar
    box_front_txt = ax_lidar.text(0, 1.42, '', color=C_FRONT_BOX, fontsize=11,
                                  ha='center', va='top', fontweight='bold', zorder=9)
    # Estado FSM — esquina inferior izquierda del panel lidar
    state_badge = ax_lidar.text(-1.45, -1.45, 'CRUCERO', color=C_OK,
                                fontsize=14, fontweight='bold',
                                ha='left', va='bottom', zorder=10,
                                bbox=dict(boxstyle='round,pad=0.3',
                                          fc=BG, ec=C_OK, lw=2.0, alpha=0.92))
    ax_lidar.legend(handles=[
        mpatches.Patch(color=C_FRONT_OK,  label='FRENTE (sin caja)'),
        mpatches.Patch(color=C_FRONT_BOX, label='FRENTE (CAJA!)'),
        mpatches.Patch(color=C_LEFT,      label='IZQ'),
        mpatches.Patch(color=C_RIGHT,     label='DER'),
    ], loc='upper right', facecolor=BG, labelcolor=C_TEXT, fontsize=8, framealpha=0.9)

    # ── Panel DER ─────────────────────────────────────────────────────────
    ax_der.axis('off')
    ax_der.set_xlim(0, 1)
    ax_der.set_ylim(0, 1)
    ax_der.set_title('PARED DERECHA', color=C_RIGHT, fontsize=12, pad=6)

    # Barra gauge horizontal
    ax_der.add_patch(mpatches.FancyBboxPatch(
        (0.06, 0.76), 0.88, 0.10, boxstyle='round,pad=0.01',
        fc='#1e2a1e', ec=BORDER, lw=1.5, zorder=1))
    ok_lo = (TARGET_DER - TOL_OK) / MAX_GAUGE
    ok_hi = (TARGET_DER + TOL_OK) / MAX_GAUGE
    ax_der.add_patch(mpatches.Rectangle(
        (0.06 + 0.88 * ok_lo, 0.76), 0.88 * (ok_hi - ok_lo), 0.10,
        fc='#1a4a1a', ec='none', zorder=2))
    tgt_x = 0.06 + 0.88 * (TARGET_DER / MAX_GAUGE)
    ax_der.plot([tgt_x, tgt_x], [0.74, 0.88], color=C_OK, lw=2.0, zorder=4)
    ax_der.text(tgt_x, 0.70, f'{int(TARGET_DER*100)} cm',
                ha='center', va='top', color=C_OK, fontsize=7)
    for cm in [0, 10, 20, 30, 40, 50, 60, 70]:
        xp = 0.06 + 0.88 * (cm / 100 / MAX_GAUGE)
        if xp > 0.94:
            break
        ax_der.text(xp, 0.89, f'{cm}', ha='center', va='bottom',
                    color=C_DIM, fontsize=6)
        ax_der.plot([xp, xp], [0.87, 0.89], color=BORDER, lw=0.8)

    bar_der = mpatches.FancyBboxPatch(
        (0.06, 0.77), 0.0, 0.08, boxstyle='round,pad=0.0',
        fc=C_OK, ec='none', zorder=3)
    ax_der.add_patch(bar_der)
    needle_der, = ax_der.plot([], [], color='white', lw=2.5, zorder=5)

    lbl_der_val    = ax_der.text(0.5, 0.48, '---', ha='center', va='center',
                                  color=C_TEXT, fontsize=34, fontweight='bold')
    lbl_der_status = ax_der.text(0.5, 0.20, '', ha='center', va='center',
                                  color=C_DIM, fontsize=10)

    der_artists = {
        'bar': bar_der, 'needle': needle_der,
        'lbl_val': lbl_der_val, 'lbl_status': lbl_der_status,
    }

    # ── Panel IZQ ─────────────────────────────────────────────────────────
    ax_izq.axis('off')
    ax_izq.set_xlim(0, 1)
    ax_izq.set_ylim(0, 1)
    ax_izq.set_title('PARED IZQUIERDA', color=C_LEFT, fontsize=12, pad=6)

    lbl_izq_val    = ax_izq.text(0.5, 0.60, '---', ha='center', va='center',
                                  color=C_LEFT, fontsize=34, fontweight='bold')
    lbl_izq_status = ax_izq.text(0.5, 0.22, '', ha='center', va='center',
                                  color=C_DIM, fontsize=11)
    # Linea indicadora del limite de repulsion (15cm)
    ax_izq.text(0.5, 0.07, f'Limite repulsion: {int(DIST_IZQ_MIN*100)} cm',
                ha='center', va='bottom', color=C_DIM, fontsize=8)

    izq_artists = {'lbl_val': lbl_izq_val, 'lbl_status': lbl_izq_status}

    # ── Velocidad ──────────────────────────────────────────────────────────
    ax_vel.set_title('Vel. lineal (m/s)', color=C_TEXT, fontsize=9, pad=4)
    ax_vel.set_xlim(0, MAX_VEL_T)
    ax_vel.set_ylim(-0.25, 0.30)
    ax_vel.axhline(0, color=BORDER, lw=0.8)
    ax_vel.set_xlabel('t (s)', color=C_DIM, fontsize=7)
    vel_line, = ax_vel.plot([], [], color=C_ALERT, lw=1.5)

    return (fig, ax_lidar, ax_der, ax_izq, ax_vel,
            lc, traj_line, box_patches, range_circ,
            alert_txt, box_front_txt, state_badge,
            der_artists, izq_artists, vel_line)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--front', type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = LidarViz(front_deg=args.front)

    plt.ion()
    (fig, ax_lidar, ax_der, ax_izq, ax_vel,
     lc, traj_line, box_patches, range_circ,
     alert_txt, box_front_txt, state_badge,
     der_artists, izq_artists, vel_line) = build_figure()
    plt.show()
    box_scan_patches = []   # rectangulo de caja detectada por scan (se borra cada frame)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.04)

            # ── Líneas LiDAR ──────────────────────────────────────────────
            if node.scan_segs:
                lc.set_segments(node.scan_segs)
                lc.set_colors(node.scan_seg_cols)
            else:
                lc.set_segments([])

            # ── Trayectoria ───────────────────────────────────────────────
            if len(node.traj_odom) > 1:
                txs = [node._odom_to_display(x, y)[0] for x, y in node.traj_odom]
                tys = [node._odom_to_display(x, y)[1] for x, y in node.traj_odom]
                traj_line.set_data(txs, tys)

            # ── Cajas censadas ────────────────────────────────────────────
            for p in box_patches:
                p.remove()
            box_patches.clear()
            for ox, oy in node.cajas_odom:
                xd, yd = node._odom_to_display(ox, oy)
                rect = mpatches.FancyBboxPatch(
                    (xd - 0.10, yd - 0.10), 0.20, 0.20,
                    boxstyle='square,pad=0', lw=1.5,
                    edgecolor=C_BOX, facecolor=C_BOX + '44', zorder=5)
                ax_lidar.add_patch(rect)
                box_patches.append(rect)

            range_circ.set_radius(min(node.range_max, 1.4))

            # ── Rectángulo de caja detectada por scan ─────────────────────
            # Se dibuja en el mapa en la posicion real del obstaculo detectado.
            # display: xd = -cy_robot (lateral), yd = cx_robot (profundidad)
            for p in box_scan_patches:
                p.remove()
            box_scan_patches.clear()
            if node.box_frente:
                xd_c = -node.box_cy          # centro x en display
                yd_c =  node.box_cx          # centro y en display
                hw   =  node.box_w / 2.0    # semiancho
                depth = 0.06                 # grosor del rectangulo en display
                rect = mpatches.FancyBboxPatch(
                    (xd_c - hw, yd_c - depth / 2), node.box_w, depth,
                    boxstyle='square,pad=0.01', lw=3,
                    edgecolor=C_FRONT_BOX, facecolor=C_FRONT_BOX + '55', zorder=7)
                ax_lidar.add_patch(rect)
                box_scan_patches.append(rect)

            # ── Alerta frontal en LiDAR ───────────────────────────────────
            if node.box_frente:
                alert_txt.set_text(f'! CAJA: {node.box_frente_dist*100:.0f} cm  ancho:{node.box_w*100:.0f} cm')
                alert_txt.set_color(C_FRONT_BOX)
                box_front_txt.set_text('▶ CAJA PERPENDICULAR DETECTADA ◀')
                box_front_txt.set_color(C_FRONT_BOX)
            else:
                alert_txt.set_text('')
                box_front_txt.set_text('')

            # ── Panel DER ─────────────────────────────────────────────────
            dr = node.d_right
            da = der_artists
            if math.isfinite(dr) and dr <= MAX_GAUGE:
                frac = min(dr / MAX_GAUGE, 1.0)
                err  = dr - TARGET_DER
                if abs(err) <= TOL_OK:
                    col_der = C_OK
                    der_status = f'EN OBJETIVO  ({err*100:+.0f} cm)'
                elif abs(err) <= 0.08:
                    col_der = C_WARN
                    der_status = f'{"LEJOS" if err > 0 else "CERCA"}  {abs(err)*100:.0f} cm'
                else:
                    col_der = C_ALERT
                    der_status = f'{"MUY LEJOS" if err > 0 else "MUY CERCA"}  {abs(err)*100:.0f} cm'

                da['bar'].set_x(0.06)
                da['bar'].set_width(0.88 * frac)
                da['bar'].set_facecolor(col_der + 'bb')
                nx = 0.06 + 0.88 * frac
                da['needle'].set_data([nx, nx], [0.74, 0.88])
                da['lbl_val'].set_text(f'{dr*100:.1f} cm')
                da['lbl_val'].set_color(col_der)
                da['lbl_status'].set_text(der_status)
                da['lbl_status'].set_color(col_der)
            else:
                da['bar'].set_width(0.0)
                da['needle'].set_data([], [])
                da['lbl_val'].set_text('---')
                da['lbl_val'].set_color(C_DIM)
                da['lbl_status'].set_text('pared no visible')
                da['lbl_status'].set_color(C_DIM)

            # ── Panel IZQ ─────────────────────────────────────────────────
            dl = node.d_left
            ia = izq_artists
            if math.isfinite(dl):
                if dl < DIST_IZQ_MIN:
                    col_izq    = C_ALERT
                    izq_status = f'! REPULSION ({dl*100:.1f} cm)'
                    ax_izq.set_facecolor('#1a0a0a')
                elif dl < DIST_IZQ_WARN:
                    col_izq    = C_WARN
                    izq_status = f'CERCA  ({dl*100:.1f} cm del limite)'
                    ax_izq.set_facecolor('#1a1208')
                else:
                    col_izq    = C_LEFT
                    izq_status = 'OK'
                    ax_izq.set_facecolor(PANEL)
                ia['lbl_val'].set_text(f'{dl*100:.1f} cm')
                ia['lbl_val'].set_color(col_izq)
                ia['lbl_status'].set_text(izq_status)
                ia['lbl_status'].set_color(col_izq)
            else:
                ia['lbl_val'].set_text('---')
                ia['lbl_val'].set_color(C_DIM)
                ia['lbl_status'].set_text('pared no visible')
                ia['lbl_status'].set_color(C_DIM)
                ax_izq.set_facecolor(PANEL)

            # ── Historial velocidad ───────────────────────────────────────
            if len(node.vel_times) > 1:
                t_now = time.time() - node._t0
                ts = [t - t_now + MAX_VEL_T for t in node.vel_times]
                vel_line.set_data(ts, list(node.vel_vals))
                ax_vel.set_xlim(0, MAX_VEL_T)

            # ── Estado FSM ────────────────────────────────────────────────
            st = node.fsm_state
            if st == 'GIRO':
                sc, bg_col = C_WARN,  '#1a1000'
            elif st == 'RODEO':
                sc, bg_col = C_RODEO, '#110a1a'
            else:
                sc, bg_col = C_OK,    PANEL
            state_badge.set_text(st)
            state_badge.set_color(sc)
            state_badge.get_bbox_patch().set_edgecolor(sc)
            ax_lidar.set_facecolor(bg_col if not node.box_frente else '#1a1000')

            fig.canvas.draw_idle()
            plt.pause(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
