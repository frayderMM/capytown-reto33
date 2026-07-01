#!/usr/bin/env python3
"""
lidar_viz_super_hud.py — Monitor visual en tiempo real del robot CapyTown.
VERSION: SUPER HUD

Mejoras visuales:
- Dashboard tipo HUD con panel principal LiDAR + tarjetas de métricas.
- Sector frontal resaltado y cambio visual cuando se detecta caja.
- Cards grandes para FRENTE, DERECHA, IZQUIERDA y COMANDOS.
- Historial de velocidad más limpio.
- Mantiene la misma lógica ROS y los mismos tópicos del archivo original.
"""

VIZ_VERSION = 'SUPER HUD v15'

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


# ──────────────────────────────────────────────────────────────────────────────
# PALETA VISUAL
# ──────────────────────────────────────────────────────────────────────────────
BG          = '#070b12'
PANEL       = '#0f1724'
PANEL_2     = '#111c2d'
BORDER      = '#26364d'
GRID        = '#1d2a3d'

C_FRONT_OK  = '#00e5ff'   # cyan: sector frontal libre
C_FRONT_BOX = '#ffb020'   # naranja: caja detectada
C_LEFT      = '#65a3ff'   # azul: pared izquierda
C_RIGHT     = '#35e087'   # verde: pared derecha
C_OTHER     = '#33465f'   # gris/azul: resto LiDAR
C_BOX       = '#ffb020'
C_RODEO     = '#b388ff'
C_TRAJ      = '#a6b3c5'
C_ARROW     = '#ffe066'
C_TEXT      = '#edf6ff'
C_DIM       = '#8fa3bd'
C_WARN      = '#ffb020'
C_ALERT     = '#ff4d6d'
C_OK        = '#35e087'


# ──────────────────────────────────────────────────────────────────────────────
# CONSTANTES
# ──────────────────────────────────────────────────────────────────────────────
FRONT_GIRO_HALF = math.radians(12.0)
SIDE_LO    = math.radians(60.0)
SIDE_HI    = math.radians(120.0)

TARGET_DER   = 0.08   # m
TOL_OK       = 0.02   # m
MAX_GAUGE    = 0.70   # m

DIST_IZQ_MIN  = 0.15  # m
DIST_IZQ_WARN = 0.25  # m

DETECT_HALF   = math.radians(20.0)
DETECT_MAX_R  = 0.45
PERP_STD_MAX  = 0.04
BOX_W_MIN     = 0.08
BOX_W_MAX     = 0.23
MIN_FRONT_PTS = 5

MAX_TRAJ    = 400
MAX_VEL_T   = 10.0


# ──────────────────────────────────────────────────────────────────────────────
# NODO ROS
# ──────────────────────────────────────────────────────────────────────────────
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
        self.box_cx          = 0.0
        self.box_cy          = 0.0
        self.box_w           = 0.0

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
        c = math.cos(-self.robot_yaw)
        s = math.sin(-self.robot_yaw)
        xb = c * dx - s * dy
        yb = s * dx + c * dy
        return -yb, xb

    def _cb_scan(self, msg: LaserScan):
        segs      = []
        seg_cols  = []
        d_f = d_l = d_r = float('inf')
        front_rf  = []
        prev_xd = prev_yd = prev_r = None

        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Saltamos de 4 en 4 puntos para que el panel sea fluido en Raspberry.
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

            if abs_af <= FRONT_GIRO_HALF:
                color = '__front__'
                d_f = min(d_f, r)
            elif SIDE_LO <= abs_af <= SIDE_HI:
                if af > 0:
                    color = C_LEFT
                    d_l = min(d_l, r)
                else:
                    color = C_RIGHT
                    d_r = min(d_r, r)
            else:
                color = C_OTHER

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

        # Detección de caja perpendicular de aprox. 25 cm.
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

        front_color = C_FRONT_BOX if self.box_frente else C_FRONT_OK
        self.scan_segs     = segs
        self.scan_seg_cols = [front_color if c == '__front__' else c for c in seg_cols]

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


# ──────────────────────────────────────────────────────────────────────────────
# HELPERS VISUALES
# ──────────────────────────────────────────────────────────────────────────────
def _style_axis(ax):
    ax.set_facecolor(PANEL)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
        sp.set_linewidth(1.2)
    ax.tick_params(colors=C_DIM, labelsize=8)


def _add_card_bg(ax, edge=BORDER):
    """Fondo tipo tarjeta dentro de un eje sin ticks."""
    ax.axis('off')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    card = mpatches.FancyBboxPatch(
        (0.02, 0.04), 0.96, 0.92,
        boxstyle='round,pad=0.02,rounding_size=0.05',
        fc=PANEL_2, ec=edge, lw=1.5, alpha=0.98, zorder=0
    )
    ax.add_patch(card)
    return card


def _metric_card(ax, title, accent, unit='cm', hint=''):
    card = _add_card_bg(ax, edge=accent)
    ax.text(0.07, 0.84, title, ha='left', va='center',
            color=accent, fontsize=10, fontweight='bold')
    value = ax.text(0.07, 0.53, '---', ha='left', va='center',
                    color=C_TEXT, fontsize=30, fontweight='bold')
    unit_txt = ax.text(0.91, 0.53, unit, ha='right', va='center',
                       color=C_DIM, fontsize=12, fontweight='bold')
    status = ax.text(0.07, 0.25, '', ha='left', va='center',
                     color=C_DIM, fontsize=10)
    hint_txt = ax.text(0.07, 0.10, hint, ha='left', va='center',
                       color=C_DIM, fontsize=8)
    return {
        'card': card,
        'value': value,
        'unit': unit_txt,
        'status': status,
        'hint': hint_txt,
    }


def _cmd_card(ax):
    card = _add_card_bg(ax, edge=C_ARROW)
    ax.text(0.07, 0.84, 'COMANDOS', ha='left', va='center',
            color=C_ARROW, fontsize=10, fontweight='bold')
    v_lin = ax.text(0.07, 0.58, 'v: ---', ha='left', va='center',
                    color=C_TEXT, fontsize=20, fontweight='bold')
    v_ang = ax.text(0.07, 0.38, 'w: ---', ha='left', va='center',
                    color=C_DIM, fontsize=14, fontweight='bold')
    corr = ax.text(0.07, 0.18, 'corr: ---', ha='left', va='center',
                   color=C_DIM, fontsize=12)
    return {'card': card, 'v_lin': v_lin, 'v_ang': v_ang, 'corr': corr}


def _set_card_state(card_art, color):
    card_art['card'].set_edgecolor(color)
    card_art['value'].set_color(color)
    card_art['status'].set_color(color)


def _fmt_cm(value_m):
    if math.isfinite(value_m):
        return f'{value_m * 100:.1f}'
    return '---'


# ──────────────────────────────────────────────────────────────────────────────
# FIGURA / DASHBOARD
# ──────────────────────────────────────────────────────────────────────────────
def build_figure():
    plt.rcParams['font.family'] = 'DejaVu Sans'

    fig = plt.figure(figsize=(15.5, 8.6), facecolor=BG)
    gs = gridspec.GridSpec(
        4, 3,
        width_ratios=[2.45, 1.0, 1.0],
        height_ratios=[0.36, 1.0, 1.0, 0.90],
        hspace=0.25,
        wspace=0.20,
        left=0.035,
        right=0.975,
        top=0.965,
        bottom=0.06,
    )

    ax_header = fig.add_subplot(gs[0, :])
    ax_lidar  = fig.add_subplot(gs[1:, 0])
    ax_front  = fig.add_subplot(gs[1, 1])
    ax_der    = fig.add_subplot(gs[1, 2])
    ax_izq    = fig.add_subplot(gs[2, 1])
    ax_cmd    = fig.add_subplot(gs[2, 2])
    ax_vel    = fig.add_subplot(gs[3, 1:])

    # Header
    ax_header.axis('off')
    ax_header.set_xlim(0, 1)
    ax_header.set_ylim(0, 1)
    ax_header.add_patch(mpatches.FancyBboxPatch(
        (0.0, 0.10), 1.0, 0.80,
        boxstyle='round,pad=0.01,rounding_size=0.03',
        fc=PANEL, ec=BORDER, lw=1.2, alpha=0.96
    ))
    ax_header.text(0.025, 0.55, 'CAPYTOWN · LiDAR REAL-TIME HUD',
                   ha='left', va='center', color=C_TEXT,
                   fontsize=18, fontweight='bold')
    ax_header.text(0.42, 0.55,
                   'scan · odom · cmd_vel · fsm_state · cajas_avistadas',
                   ha='left', va='center', color=C_DIM, fontsize=10)
    header_state = ax_header.text(
        0.965, 0.55, 'CRUCERO', ha='right', va='center',
        color=C_OK, fontsize=13, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.35', fc=BG, ec=C_OK, lw=1.8, alpha=0.95)
    )

    # LiDAR principal
    _style_axis(ax_lidar)
    ax_lidar.set_xlim(-1.55, 1.55)
    ax_lidar.set_ylim(-1.55, 1.55)
    ax_lidar.set_aspect('equal')
    ax_lidar.grid(True, color=GRID, lw=0.55, ls='--', alpha=0.9)
    ax_lidar.set_title(f'MAPA LiDAR 360°  ·  {VIZ_VERSION}',
                       color=C_TEXT, fontsize=13, fontweight='bold', pad=10)
    ax_lidar.set_xlabel('← IZQUIERDA        DERECHA →', color=C_DIM, fontsize=8)
    ax_lidar.set_ylabel('↑ FRENTE', color=C_DIM, fontsize=8)

    # Sectores sutiles de referencia. En matplotlib, 90° apunta hacia arriba.
    front_wedge = mpatches.Wedge((0, 0), 1.45, 78, 102,
                                 facecolor=C_FRONT_OK, edgecolor=C_FRONT_OK,
                                 alpha=0.08, lw=1.0, zorder=1)
    left_wedge = mpatches.Wedge((0, 0), 1.45, 102, 150,
                                facecolor=C_LEFT, edgecolor='none', alpha=0.04, zorder=1)
    right_wedge = mpatches.Wedge((0, 0), 1.45, 30, 78,
                                 facecolor=C_RIGHT, edgecolor='none', alpha=0.04, zorder=1)
    ax_lidar.add_patch(front_wedge)
    ax_lidar.add_patch(left_wedge)
    ax_lidar.add_patch(right_wedge)

    for d, label in [(0.3, '30 cm'), (0.5, '50 cm'), (1.0, '1 m'), (1.5, '1.5 m')]:
        ax_lidar.add_patch(plt.Circle((0, 0), d, color=BORDER,
                                      fill=False, lw=0.8, ls=':', alpha=0.95, zorder=2))
        ax_lidar.text(0.03, d + 0.025, label, color=C_DIM, fontsize=7, alpha=0.85)

    # Robot en el centro
    ax_lidar.scatter([0], [0], s=95, c=C_TEXT, edgecolor=BG, linewidth=1.0, zorder=7)
    ax_lidar.annotate('', xy=(0, 0.28), xytext=(0, 0),
                      arrowprops=dict(arrowstyle='-|>', color=C_ARROW, lw=3.0), zorder=8)
    ax_lidar.text(0.05, -0.08, 'robot', color=C_DIM, fontsize=8, zorder=8)

    lc = LineCollection([], linewidths=3.2, alpha=0.98, zorder=5)
    ax_lidar.add_collection(lc)

    traj_line, = ax_lidar.plot([], [], color=C_TRAJ, lw=1.4, alpha=0.65, zorder=3)

    range_circ = plt.Circle((0, 0), 1.45, color='#27405f',
                            fill=False, lw=1.0, ls='--', alpha=0.9, zorder=2)
    ax_lidar.add_patch(range_circ)

    alert_txt = ax_lidar.text(0, -1.43, '', color=C_ALERT, fontsize=12,
                              ha='center', va='bottom', fontweight='bold', zorder=9,
                              bbox=dict(boxstyle='round,pad=0.25', fc=BG, ec='none', alpha=0.70))
    box_front_txt = ax_lidar.text(0, 1.43, '', color=C_FRONT_BOX, fontsize=12,
                                  ha='center', va='top', fontweight='bold', zorder=9,
                                  bbox=dict(boxstyle='round,pad=0.25', fc=BG, ec='none', alpha=0.70))

    state_badge = ax_lidar.text(-1.45, -1.45, 'CRUCERO', color=C_OK,
                                fontsize=13, fontweight='bold',
                                ha='left', va='bottom', zorder=10,
                                bbox=dict(boxstyle='round,pad=0.35',
                                          fc=BG, ec=C_OK, lw=2.0, alpha=0.95))

    ax_lidar.legend(handles=[
        mpatches.Patch(color=C_FRONT_OK,  label='Frente libre'),
        mpatches.Patch(color=C_FRONT_BOX, label='Caja frontal'),
        mpatches.Patch(color=C_LEFT,      label='Sector izquierdo'),
        mpatches.Patch(color=C_RIGHT,     label='Sector derecho'),
    ], loc='upper right', facecolor=BG, edgecolor=BORDER,
       labelcolor=C_TEXT, fontsize=8, framealpha=0.92)

    box_patches = []

    # Cards laterales
    front_card = _metric_card(ax_front, 'FRENTE', C_FRONT_OK, unit='cm', hint='Sector ±12°')
    der_card   = _metric_card(ax_der,   'PARED DERECHA', C_RIGHT, unit='cm', hint=f'Objetivo: {int(TARGET_DER*100)} cm')
    izq_card   = _metric_card(ax_izq,   'PARED IZQUIERDA', C_LEFT, unit='cm', hint=f'Repulsión < {int(DIST_IZQ_MIN*100)} cm')
    cmd_card   = _cmd_card(ax_cmd)

    # Velocidad
    _style_axis(ax_vel)
    ax_vel.set_title('HISTORIAL DE VELOCIDAD LINEAL', color=C_TEXT,
                     fontsize=10, fontweight='bold', pad=6)
    ax_vel.set_xlim(0, MAX_VEL_T)
    ax_vel.set_ylim(-0.25, 0.30)
    ax_vel.grid(True, color=GRID, lw=0.5, ls='--', alpha=0.8)
    ax_vel.axhline(0, color=BORDER, lw=0.9)
    ax_vel.set_xlabel('últimos 10 s', color=C_DIM, fontsize=8)
    ax_vel.set_ylabel('m/s', color=C_DIM, fontsize=8)
    vel_line, = ax_vel.plot([], [], color=C_ALERT, lw=2.0)

    artists = {
        'header_state': header_state,
        'front_wedge': front_wedge,
        'front_card': front_card,
        'der_card': der_card,
        'izq_card': izq_card,
        'cmd_card': cmd_card,
    }

    return (fig, ax_lidar, ax_vel,
            lc, traj_line, box_patches, range_circ,
            alert_txt, box_front_txt, state_badge,
            artists, vel_line)


# ──────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ──────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--front', type=float, default=180.0,
                        help='Ángulo que representa el frente del robot en grados. Default: 180')
    args = parser.parse_args()

    rclpy.init()
    node = LidarViz(front_deg=args.front)

    plt.ion()
    (fig, ax_lidar, ax_vel,
     lc, traj_line, box_patches, range_circ,
     alert_txt, box_front_txt, state_badge,
     artists, vel_line) = build_figure()

    plt.show()
    box_scan_patches = []

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.04)

            # ── LiDAR ────────────────────────────────────────────────────
            if node.scan_segs:
                lc.set_segments(node.scan_segs)
                lc.set_colors(node.scan_seg_cols)
            else:
                lc.set_segments([])

            # ── Trayectoria odométrica ───────────────────────────────────
            if len(node.traj_odom) > 1:
                txs = [node._odom_to_display(x, y)[0] for x, y in node.traj_odom]
                tys = [node._odom_to_display(x, y)[1] for x, y in node.traj_odom]
                traj_line.set_data(txs, tys)

            # ── Cajas censadas por odometría ─────────────────────────────
            for p in box_patches:
                p.remove()
            box_patches.clear()

            for ox, oy in node.cajas_odom:
                xd, yd = node._odom_to_display(ox, oy)
                rect = mpatches.FancyBboxPatch(
                    (xd - 0.10, yd - 0.10), 0.20, 0.20,
                    boxstyle='round,pad=0.01,rounding_size=0.02',
                    lw=1.8, edgecolor=C_BOX, facecolor=C_BOX + '44', zorder=6)
                ax_lidar.add_patch(rect)
                box_patches.append(rect)

            range_circ.set_radius(min(node.range_max, 1.45))

            # ── Caja detectada por el scan frontal ───────────────────────
            for p in box_scan_patches:
                p.remove()
            box_scan_patches.clear()

            if node.box_frente:
                xd_c = -node.box_cy
                yd_c =  node.box_cx
                hw   =  node.box_w / 2.0
                depth = 0.07

                rect = mpatches.FancyBboxPatch(
                    (xd_c - hw, yd_c - depth / 2), node.box_w, depth,
                    boxstyle='round,pad=0.01,rounding_size=0.015',
                    lw=3.0, edgecolor=C_FRONT_BOX,
                    facecolor=C_FRONT_BOX + '66', zorder=9)
                ax_lidar.add_patch(rect)
                box_scan_patches.append(rect)

            # ── Estado visual frontal ────────────────────────────────────
            front_card = artists['front_card']
            if node.box_frente:
                front_value = node.box_frente_dist
                front_card['value'].set_text(_fmt_cm(front_value))
                front_card['unit'].set_text('cm')
                front_card['status'].set_text(f'CAJA DETECTADA · ancho {node.box_w*100:.0f} cm')
                _set_card_state(front_card, C_FRONT_BOX)
                artists['front_wedge'].set_facecolor(C_FRONT_BOX)
                artists['front_wedge'].set_edgecolor(C_FRONT_BOX)
                artists['front_wedge'].set_alpha(0.18)
                alert_txt.set_text(f'⚠ CAJA FRONTAL: {node.box_frente_dist*100:.0f} cm · ancho {node.box_w*100:.0f} cm')
                alert_txt.set_color(C_FRONT_BOX)
                box_front_txt.set_text('OBJETO PERPENDICULAR DETECTADO')
                box_front_txt.set_color(C_FRONT_BOX)
            else:
                front_card['value'].set_text(_fmt_cm(node.d_front))
                front_card['unit'].set_text('cm')
                front_card['status'].set_text('LIBRE' if math.isfinite(node.d_front) else 'sin lectura frontal')
                _set_card_state(front_card, C_FRONT_OK)
                artists['front_wedge'].set_facecolor(C_FRONT_OK)
                artists['front_wedge'].set_edgecolor(C_FRONT_OK)
                artists['front_wedge'].set_alpha(0.08)
                alert_txt.set_text('')
                box_front_txt.set_text('')

            # ── Panel pared derecha ──────────────────────────────────────
            dr = node.d_right
            der_card = artists['der_card']
            if math.isfinite(dr) and dr <= MAX_GAUGE:
                err = dr - TARGET_DER
                der_card['value'].set_text(_fmt_cm(dr))
                if abs(err) <= TOL_OK:
                    col_der = C_OK
                    status = f'EN OBJETIVO ({err*100:+.0f} cm)'
                elif abs(err) <= 0.08:
                    col_der = C_WARN
                    status = f'{"LEJOS" if err > 0 else "CERCA"} · error {abs(err)*100:.0f} cm'
                else:
                    col_der = C_ALERT
                    status = f'{"MUY LEJOS" if err > 0 else "MUY CERCA"} · error {abs(err)*100:.0f} cm'
                der_card['status'].set_text(status)
                _set_card_state(der_card, col_der)
            else:
                der_card['value'].set_text('---')
                der_card['status'].set_text('pared no visible')
                _set_card_state(der_card, C_DIM)

            # ── Panel pared izquierda ────────────────────────────────────
            dl = node.d_left
            izq_card = artists['izq_card']
            if math.isfinite(dl):
                izq_card['value'].set_text(_fmt_cm(dl))
                if dl < DIST_IZQ_MIN:
                    col_izq = C_ALERT
                    status = f'REPULSIÓN · {dl*100:.1f} cm'
                elif dl < DIST_IZQ_WARN:
                    col_izq = C_WARN
                    status = f'CERCA DEL LÍMITE · {dl*100:.1f} cm'
                else:
                    col_izq = C_LEFT
                    status = 'OK'
                izq_card['status'].set_text(status)
                _set_card_state(izq_card, col_izq)
            else:
                izq_card['value'].set_text('---')
                izq_card['status'].set_text('pared no visible')
                _set_card_state(izq_card, C_DIM)

            # ── Card comandos ────────────────────────────────────────────
            cmd_card = artists['cmd_card']
            cmd_card['v_lin'].set_text(f'v: {node.vel_lin:+.3f} m/s')
            cmd_card['v_ang'].set_text(f'w: {node.vel_ang:+.3f} rad/s')
            cmd_card['corr'].set_text(f'corr lateral: {node.lat_corr:+.3f}')

            if abs(node.vel_ang) > 0.35:
                cmd_card['card'].set_edgecolor(C_WARN)
                cmd_card['v_ang'].set_color(C_WARN)
            else:
                cmd_card['card'].set_edgecolor(C_ARROW)
                cmd_card['v_ang'].set_color(C_DIM)

            # ── Historial velocidad ──────────────────────────────────────
            if len(node.vel_times) > 1:
                t_now = time.time() - node._t0
                ts = [t - t_now + MAX_VEL_T for t in node.vel_times]
                vel_line.set_data(ts, list(node.vel_vals))
                ax_vel.set_xlim(0, MAX_VEL_T)

            # ── Estado FSM ───────────────────────────────────────────────
            st = node.fsm_state.strip() if node.fsm_state else '---'
            if st == 'GIRO':
                sc, bg_col = C_WARN, '#1f1607'
            elif st == 'RODEO':
                sc, bg_col = C_RODEO, '#160d25'
            else:
                sc, bg_col = C_OK, PANEL

            state_badge.set_text(st)
            state_badge.set_color(sc)
            state_badge.get_bbox_patch().set_edgecolor(sc)

            artists['header_state'].set_text(st)
            artists['header_state'].set_color(sc)
            artists['header_state'].get_bbox_patch().set_edgecolor(sc)

            ax_lidar.set_facecolor('#1f1607' if node.box_frente else bg_col)

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
