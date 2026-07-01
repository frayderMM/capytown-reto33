#!/usr/bin/env python3
"""
lidar_viz_super_hud_v2.py — Monitor visual en tiempo real del robot CapyTown.
VERSION: ULTRA UI

Objetivo:
- Mantener la misma lógica ROS del visualizador original.
- Mantener un tamaño de ventana contenido (12x7).
- Mejorar totalmente la UI: cards más limpias, LiDAR más legible,
  puntos más bonitos y robot dibujado como icono de carrito.

Tópicos usados:
  /scan
  /odom_raw
  /cmd_vel
  /lateral_correction
  /cajas_avistadas
  /fsm_state
"""

VIZ_VERSION = 'ULTRA UI v16'

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
from matplotlib.lines import Line2D
import numpy as np


# ──────────────────────────────────────────────────────────────────────────────
# PALETA
# ──────────────────────────────────────────────────────────────────────────────
BG          = '#0b1018'
PANEL       = '#121a26'
PANEL_2     = '#0f1722'
BORDER      = '#263345'
GRID        = '#223145'
CARD_BG     = '#111927'
CARD_SOFT   = '#172334'

C_FRONT_OK  = '#3dd9eb'
C_FRONT_BOX = '#ffb545'
C_LEFT      = '#69a7ff'
C_RIGHT     = '#40df8d'
C_OTHER     = '#52647a'
C_BOX       = '#ffb545'
C_RODEO     = '#b88cff'
C_TRAJ      = '#b0bac6'
C_ARROW     = '#ffd964'
C_TEXT      = '#eef5ff'
C_DIM       = '#92a6bf'
C_WARN      = '#ffb545'
C_ALERT     = '#ff5f7e'
C_OK        = '#40df8d'
C_CAR       = '#f7fafc'
C_CAR_TRIM  = '#2a3340'
C_CAR_GLASS = '#98d9ff'


# ──────────────────────────────────────────────────────────────────────────────
# CONSTANTES
# ──────────────────────────────────────────────────────────────────────────────
FRONT_GIRO_HALF = math.radians(12.0)
SIDE_LO    = math.radians(60.0)
SIDE_HI    = math.radians(120.0)

TARGET_DER   = 0.08
TOL_OK       = 0.02
MAX_GAUGE    = 0.70

DIST_IZQ_MIN  = 0.15
DIST_IZQ_WARN = 0.25

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

        self.scan_segs = []
        self.scan_seg_cols = []
        self.scan_pts = {
            'front': [],
            'left': [],
            'right': [],
            'other': [],
        }

        self.d_front = float('inf')
        self.d_left = float('inf')
        self.d_right = float('inf')
        self.range_min = 0.0
        self.range_max = 8.0

        self.box_frente = False
        self.box_frente_dist = float('inf')
        self.box_cx = 0.0
        self.box_cy = 0.0
        self.box_w = 0.0

        self.fsm_state = 'CRUCERO'

        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.lat_corr = 0.0
        self.cajas_odom = []

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.traj_odom = deque(maxlen=MAX_TRAJ)

        self.vel_times = deque()
        self.vel_vals = deque()
        self._t0 = time.time()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos)
        self.create_subscription(Odometry, '/odom_raw', self._cb_odom, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd, 10)
        self.create_subscription(Float32, '/lateral_correction', self._cb_lat, 10)
        self.create_subscription(PoseArray, '/cajas_avistadas', self._cb_cajas, 10)
        self.create_subscription(String, '/fsm_state', self._cb_estado, 10)

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
        segs = []
        seg_cols = []
        d_f = d_l = d_r = float('inf')
        front_rf = []
        prev_xd = prev_yd = prev_r = None

        pts_front = []
        pts_left = []
        pts_right = []
        pts_other = []

        self.range_min = msg.range_min
        self.range_max = msg.range_max

        for i in range(0, len(msg.ranges), 4):
            r = msg.ranges[i]
            if not math.isfinite(r) or r == 0.0:
                prev_xd = prev_yd = prev_r = None
                continue
            if r < msg.range_min or r > msg.range_max:
                prev_xd = prev_yd = prev_r = None
                continue

            theta = msg.angle_min + i * msg.angle_increment
            af = math.atan2(math.sin(theta - self.front_rad),
                            math.cos(theta - self.front_rad))
            abs_af = abs(af)

            if abs_af <= FRONT_GIRO_HALF:
                color = '__front__'
                d_f = min(d_f, r)
                target_bucket = pts_front
            elif SIDE_LO <= abs_af <= SIDE_HI:
                if af > 0:
                    color = C_LEFT
                    d_l = min(d_l, r)
                    target_bucket = pts_left
                else:
                    color = C_RIGHT
                    d_r = min(d_r, r)
                    target_bucket = pts_right
            else:
                color = C_OTHER
                target_bucket = pts_other

            if abs_af <= DETECT_HALF and r <= DETECT_MAX_R:
                front_rf.append((r * math.cos(af), r * math.sin(af)))

            xd, yd = self._sensor_to_display(theta, r)
            target_bucket.append((xd, yd))

            if (prev_xd is not None and prev_r is not None
                    and abs(r - prev_r) < 0.20
                    and math.hypot(xd - prev_xd, yd - prev_yd) < 0.30):
                segs.append([(prev_xd, prev_yd), (xd, yd)])
                seg_cols.append(color)

            prev_xd, prev_yd, prev_r = xd, yd, r

        self.d_front = d_f
        self.d_left = d_l
        self.d_right = d_r

        self.box_frente = False
        self.box_frente_dist = float('inf')
        self.box_cx = self.box_cy = self.box_w = 0.0
        if len(front_rf) >= MIN_FRONT_PTS:
            xs = [p[0] for p in front_rf]
            ys = [p[1] for p in front_rf]
            n = len(xs)
            mx = sum(xs) / n
            std_x = math.sqrt(sum((x - mx) ** 2 for x in xs) / n)
            y_spread = max(ys) - min(ys)
            if std_x < PERP_STD_MAX and BOX_W_MIN <= y_spread <= BOX_W_MAX:
                self.box_frente = True
                self.box_frente_dist = mx
                self.box_cx = mx
                self.box_cy = (max(ys) + min(ys)) / 2.0
                self.box_w = y_spread

        front_color = C_FRONT_BOX if self.box_frente else C_FRONT_OK
        self.scan_segs = segs
        self.scan_seg_cols = [front_color if c == '__front__' else c for c in seg_cols]
        self.scan_pts = {
            'front': pts_front,
            'left': pts_left,
            'right': pts_right,
            'other': pts_other,
        }

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
        sp.set_linewidth(1.15)
    ax.tick_params(colors=C_DIM, labelsize=8)


def _add_card_shell(ax, edge_color=BORDER):
    ax.axis('off')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    shadow = mpatches.FancyBboxPatch(
        (0.035, 0.045), 0.93, 0.89,
        boxstyle='round,pad=0.02,rounding_size=0.045',
        fc='#09111b', ec='none', alpha=0.55, zorder=0
    )
    card = mpatches.FancyBboxPatch(
        (0.02, 0.06), 0.93, 0.89,
        boxstyle='round,pad=0.02,rounding_size=0.045',
        fc=CARD_BG, ec=edge_color, lw=1.4, zorder=1
    )
    ax.add_patch(shadow)
    ax.add_patch(card)
    return card


def _metric_card(ax, title, accent, hint=''):
    card = _add_card_shell(ax, accent)
    bullet = mpatches.Circle((0.11, 0.82), 0.018, fc=accent, ec='none', zorder=2)
    ax.add_patch(bullet)
    title_t = ax.text(0.16, 0.82, title, ha='left', va='center', color=accent,
                      fontsize=10.5, fontweight='bold', zorder=3)
    value = ax.text(0.10, 0.56, '---', ha='left', va='center', color=C_TEXT,
                    fontsize=28, fontweight='bold', zorder=3)
    unit = ax.text(0.83, 0.56, 'cm', ha='right', va='center', color=C_DIM,
                   fontsize=11, fontweight='bold', zorder=3)
    status = ax.text(0.10, 0.28, '', ha='left', va='center', color=C_DIM,
                     fontsize=9.7, zorder=3)
    hint_t = ax.text(0.10, 0.12, hint, ha='left', va='center', color=C_DIM,
                     fontsize=8, zorder=3)
    bar_bg = mpatches.FancyBboxPatch(
        (0.10, 0.39), 0.73, 0.06,
        boxstyle='round,pad=0.01,rounding_size=0.02',
        fc=CARD_SOFT, ec='none', zorder=2
    )
    bar_fg = mpatches.FancyBboxPatch(
        (0.10, 0.39), 0.0, 0.06,
        boxstyle='round,pad=0.01,rounding_size=0.02',
        fc=accent, ec='none', zorder=3, alpha=0.9
    )
    ax.add_patch(bar_bg)
    ax.add_patch(bar_fg)
    return {
        'card': card,
        'bullet': bullet,
        'title': title_t,
        'value': value,
        'unit': unit,
        'status': status,
        'hint': hint_t,
        'bar_fg': bar_fg,
        'accent': accent,
    }


def _cmd_card(ax):
    card = _add_card_shell(ax, C_ARROW)
    ax.text(0.16, 0.82, 'COMANDOS', ha='left', va='center', color=C_ARROW,
            fontsize=10.5, fontweight='bold')
    ax.add_patch(mpatches.Circle((0.11, 0.82), 0.018, fc=C_ARROW, ec='none'))
    v_lin = ax.text(0.10, 0.60, 'v: ---', ha='left', va='center', color=C_TEXT,
                    fontsize=17, fontweight='bold')
    v_ang = ax.text(0.10, 0.42, 'ω: ---', ha='left', va='center', color=C_DIM,
                    fontsize=13, fontweight='bold')
    corr = ax.text(0.10, 0.22, 'corr: ---', ha='left', va='center', color=C_DIM,
                   fontsize=10.5)
    return {'card': card, 'v_lin': v_lin, 'v_ang': v_ang, 'corr': corr}


def _set_card_visual(card_art, color):
    card_art['card'].set_edgecolor(color)
    card_art['bullet'].set_facecolor(color)
    card_art['title'].set_color(color)
    card_art['value'].set_color(color)
    card_art['status'].set_color(color)
    card_art['bar_fg'].set_facecolor(color)


def _set_card_progress(card_art, frac):
    frac = max(0.0, min(1.0, frac))
    card_art['bar_fg'].set_width(0.73 * frac)


def _fmt_cm(value_m):
    return f'{value_m * 100:.1f}' if math.isfinite(value_m) else '---'


def _scatter_offsets(points):
    if not points:
        return np.empty((0, 2))
    return np.array(points)


def _draw_robot_cart(ax):
    # Carrito visto desde arriba, fijo mirando hacia arriba.
    body = mpatches.FancyBboxPatch(
        (-0.095, -0.11), 0.19, 0.28,
        boxstyle='round,pad=0.01,rounding_size=0.04',
        fc=C_CAR, ec=C_CAR_TRIM, lw=1.8, zorder=10
    )
    cabin = mpatches.FancyBboxPatch(
        (-0.06, -0.03), 0.12, 0.12,
        boxstyle='round,pad=0.01,rounding_size=0.025',
        fc=C_CAR_GLASS, ec=C_CAR_TRIM, lw=1.2, zorder=11
    )
    hood = mpatches.FancyBboxPatch(
        (-0.05, 0.09), 0.10, 0.05,
        boxstyle='round,pad=0.005,rounding_size=0.018',
        fc='#d8e2f0', ec=C_CAR_TRIM, lw=1.0, zorder=11
    )

    wheel_specs = [
        (-0.11, -0.07), (-0.11, 0.07),
        (0.09, -0.07), (0.09, 0.07),
    ]
    wheels = [mpatches.Rectangle((x, y), 0.02, 0.05, fc=C_CAR_TRIM, ec='none', zorder=9)
              for x, y in wheel_specs]

    headlight_l = mpatches.Circle((-0.035, 0.145), 0.009, fc=C_ARROW, ec='none', zorder=12)
    headlight_r = mpatches.Circle((0.035, 0.145), 0.009, fc=C_ARROW, ec='none', zorder=12)
    rear_l = mpatches.Circle((-0.035, -0.10), 0.0085, fc=C_ALERT, ec='none', zorder=12)
    rear_r = mpatches.Circle((0.035, -0.10), 0.0085, fc=C_ALERT, ec='none', zorder=12)
    nose = Line2D([0, 0], [0.17, 0.24], color=C_ARROW, lw=2.6, zorder=12)

    patches = [body, cabin, hood, headlight_l, headlight_r, rear_l, rear_r]
    for p in wheels + patches:
        ax.add_patch(p)
    ax.add_line(nose)
    return {'body': body, 'cabin': cabin, 'hood': hood, 'wheels': wheels, 'nose': nose}


# ──────────────────────────────────────────────────────────────────────────────
# FIGURA
# ──────────────────────────────────────────────────────────────────────────────
def build_figure():
    plt.rcParams['font.family'] = 'DejaVu Sans'

    # Mismo tamaño contenido pedido por el usuario
    fig = plt.figure(figsize=(12, 7), facecolor=BG)
    gs = gridspec.GridSpec(
        4, 3,
        width_ratios=[1.65, 1.65, 1.05],
        height_ratios=[0.30, 1.0, 1.0, 0.95],
        hspace=0.22,
        wspace=0.20,
        left=0.04,
        right=0.975,
        top=0.96,
        bottom=0.07,
    )

    ax_header = fig.add_subplot(gs[0, :])
    ax_lidar = fig.add_subplot(gs[1:, :2])
    ax_front = fig.add_subplot(gs[1, 2])
    ax_der = fig.add_subplot(gs[2, 2])
    ax_info = fig.add_subplot(gs[3, 2])
    ax_vel = fig.add_subplot(gs[3, :2])

    # Header
    ax_header.axis('off')
    ax_header.set_xlim(0, 1)
    ax_header.set_ylim(0, 1)
    ax_header.add_patch(mpatches.FancyBboxPatch(
        (0.0, 0.10), 1.0, 0.80,
        boxstyle='round,pad=0.01,rounding_size=0.03',
        fc=PANEL, ec=BORDER, lw=1.2
    ))
    ax_header.text(0.02, 0.56, 'CAPYTOWN · LIDAR MONITOR', ha='left', va='center',
                   color=C_TEXT, fontsize=17, fontweight='bold')
    ax_header.text(0.31, 0.56, f'{VIZ_VERSION}  ·  scan / odom / cmd_vel / fsm_state',
                   ha='left', va='center', color=C_DIM, fontsize=9.5)
    header_state = ax_header.text(
        0.97, 0.56, 'CRUCERO', ha='right', va='center',
        color=C_OK, fontsize=12.5, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.35', fc=BG, ec=C_OK, lw=1.7, alpha=0.96)
    )

    # LiDAR
    _style_axis(ax_lidar)
    ax_lidar.set_xlim(-1.55, 1.55)
    ax_lidar.set_ylim(-1.55, 1.55)
    ax_lidar.set_aspect('equal')
    ax_lidar.grid(True, color=GRID, lw=0.55, ls='--', alpha=0.95)
    ax_lidar.set_title('MAPA LIDAR 360°', color=C_TEXT, fontsize=13, fontweight='bold', pad=10)
    ax_lidar.set_xlabel('← IZQUIERDA                                DERECHA →', color=C_DIM, fontsize=8)
    ax_lidar.set_ylabel('↑ FRENTE', color=C_DIM, fontsize=8)

    front_wedge = mpatches.Wedge((0, 0), 1.47, 78, 102, facecolor=C_FRONT_OK,
                                 edgecolor=C_FRONT_OK, alpha=0.08, lw=1.0, zorder=1)
    left_wedge = mpatches.Wedge((0, 0), 1.47, 102, 150, facecolor=C_LEFT,
                                edgecolor='none', alpha=0.035, zorder=1)
    right_wedge = mpatches.Wedge((0, 0), 1.47, 30, 78, facecolor=C_RIGHT,
                                 edgecolor='none', alpha=0.035, zorder=1)
    ax_lidar.add_patch(front_wedge)
    ax_lidar.add_patch(left_wedge)
    ax_lidar.add_patch(right_wedge)

    for d, label in [(0.3, '30 cm'), (0.5, '50 cm'), (1.0, '1 m'), (1.5, '1.5 m')]:
        ring = plt.Circle((0, 0), d, color=BORDER, fill=False, lw=0.8, ls=':', alpha=0.95, zorder=2)
        ax_lidar.add_patch(ring)
        ax_lidar.text(0.03, d + 0.025, label, color=C_DIM, fontsize=7, alpha=0.85)

    range_circ = plt.Circle((0, 0), 1.45, color='#29405a', fill=False,
                            lw=1.0, ls='--', alpha=0.85, zorder=2)
    ax_lidar.add_patch(range_circ)

    # Trazos y puntos LiDAR
    lc = LineCollection([], linewidths=1.8, alpha=0.25, zorder=4)
    ax_lidar.add_collection(lc)

    sc_other = ax_lidar.scatter([], [], s=7, c=C_OTHER, alpha=0.65, edgecolors='none', zorder=5)
    sc_left = ax_lidar.scatter([], [], s=13, c=C_LEFT, alpha=0.9, edgecolors='none', zorder=6)
    sc_right = ax_lidar.scatter([], [], s=13, c=C_RIGHT, alpha=0.9, edgecolors='none', zorder=6)
    sc_front = ax_lidar.scatter([], [], s=22, c=C_FRONT_OK, alpha=0.95, edgecolors='none', zorder=7)

    traj_line, = ax_lidar.plot([], [], color=C_TRAJ, lw=1.35, alpha=0.65, zorder=3)

    robot_art = _draw_robot_cart(ax_lidar)
    ax_lidar.text(0.0, -0.18, 'robot', color=C_DIM, fontsize=8, ha='center', va='top', zorder=12)

    alert_txt = ax_lidar.text(0, -1.43, '', color=C_ALERT, fontsize=11.2,
                              ha='center', va='bottom', fontweight='bold', zorder=12,
                              bbox=dict(boxstyle='round,pad=0.28', fc=BG, ec='none', alpha=0.78))
    box_front_txt = ax_lidar.text(0, 1.43, '', color=C_FRONT_BOX, fontsize=11,
                                  ha='center', va='top', fontweight='bold', zorder=12,
                                  bbox=dict(boxstyle='round,pad=0.28', fc=BG, ec='none', alpha=0.78))
    state_badge = ax_lidar.text(-1.45, -1.45, 'CRUCERO', color=C_OK, fontsize=12.5,
                                fontweight='bold', ha='left', va='bottom', zorder=13,
                                bbox=dict(boxstyle='round,pad=0.34', fc=BG, ec=C_OK, lw=1.9, alpha=0.96))

    ax_lidar.legend(handles=[
        mpatches.Patch(color=C_FRONT_OK, label='Frente'),
        mpatches.Patch(color=C_LEFT, label='Izquierda'),
        mpatches.Patch(color=C_RIGHT, label='Derecha'),
        mpatches.Patch(color=C_FRONT_BOX, label='Caja detectada'),
    ], loc='upper right', facecolor=BG, edgecolor=BORDER, labelcolor=C_TEXT,
       fontsize=7.8, framealpha=0.92)

    box_patches = []

    # Cards
    front_card = _metric_card(ax_front, 'FRENTE', C_FRONT_OK, hint='Lectura directa del sector frontal')
    der_card = _metric_card(ax_der, 'PARED DERECHA', C_RIGHT, hint='Objetivo = 8 cm')

    # Info panel inferior derecho: izquierda + comandos en una sola tarjeta
    ax_info.axis('off')
    ax_info.set_xlim(0, 1)
    ax_info.set_ylim(0, 1)
    info_card = _add_card_shell(ax_info, BORDER)
    ax_info.text(0.12, 0.84, 'IZQUIERDA', ha='left', va='center', color=C_LEFT,
                 fontsize=10.5, fontweight='bold')
    ax_info.add_patch(mpatches.Circle((0.08, 0.84), 0.017, fc=C_LEFT, ec='none'))
    izq_value = ax_info.text(0.08, 0.64, '---', ha='left', va='center', color=C_TEXT,
                             fontsize=24, fontweight='bold')
    izq_unit = ax_info.text(0.73, 0.64, 'cm', ha='right', va='center', color=C_DIM,
                            fontsize=10.5, fontweight='bold')
    izq_status = ax_info.text(0.08, 0.49, '', ha='left', va='center', color=C_DIM,
                              fontsize=9.2)
    ax_info.plot([0.08, 0.90], [0.39, 0.39], color=BORDER, lw=1.0)

    cmd_card = {
        'card': info_card,
        'v_lin': ax_info.text(0.08, 0.28, 'v: ---', ha='left', va='center', color=C_TEXT,
                              fontsize=12.5, fontweight='bold'),
        'v_ang': ax_info.text(0.08, 0.18, 'ω: ---', ha='left', va='center', color=C_DIM,
                              fontsize=10.5, fontweight='bold'),
        'corr': ax_info.text(0.08, 0.09, 'corr: ---', ha='left', va='center', color=C_DIM,
                             fontsize=9.2),
    }

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
        'izq_value': izq_value,
        'izq_unit': izq_unit,
        'izq_status': izq_status,
        'info_card': info_card,
        'cmd_card': cmd_card,
        'sc_other': sc_other,
        'sc_left': sc_left,
        'sc_right': sc_right,
        'sc_front': sc_front,
        'robot_art': robot_art,
    }

    return (
        fig, ax_lidar, ax_vel,
        lc, traj_line, box_patches, range_circ,
        alert_txt, box_front_txt, state_badge,
        artists, vel_line,
    )


# ──────────────────────────────────────────────────────────────────────────────
# MAIN
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

            # LiDAR líneas suaves
            if node.scan_segs:
                lc.set_segments(node.scan_segs)
                lc.set_colors(node.scan_seg_cols)
            else:
                lc.set_segments([])

            # LiDAR puntos
            artists['sc_other'].set_offsets(_scatter_offsets(node.scan_pts['other']))
            artists['sc_left'].set_offsets(_scatter_offsets(node.scan_pts['left']))
            artists['sc_right'].set_offsets(_scatter_offsets(node.scan_pts['right']))
            artists['sc_front'].set_offsets(_scatter_offsets(node.scan_pts['front']))
            artists['sc_front'].set_color(C_FRONT_BOX if node.box_frente else C_FRONT_OK)

            # Trayectoria
            if len(node.traj_odom) > 1:
                txs = [node._odom_to_display(x, y)[0] for x, y in node.traj_odom]
                tys = [node._odom_to_display(x, y)[1] for x, y in node.traj_odom]
                traj_line.set_data(txs, tys)
            else:
                traj_line.set_data([], [])

            # Cajas odométricas
            for p in box_patches:
                p.remove()
            box_patches.clear()
            for ox, oy in node.cajas_odom:
                xd, yd = node._odom_to_display(ox, oy)
                rect = mpatches.FancyBboxPatch(
                    (xd - 0.10, yd - 0.10), 0.20, 0.20,
                    boxstyle='round,pad=0.01,rounding_size=0.02',
                    lw=1.7, edgecolor=C_BOX, facecolor=C_BOX + '40', zorder=8
                )
                ax_lidar.add_patch(rect)
                box_patches.append(rect)

            range_circ.set_radius(min(node.range_max, 1.45))

            # Caja detectada frontal por scan
            for p in box_scan_patches:
                p.remove()
            box_scan_patches.clear()
            if node.box_frente:
                xd_c = -node.box_cy
                yd_c = node.box_cx
                depth = 0.07
                rect = mpatches.FancyBboxPatch(
                    (xd_c - node.box_w / 2.0, yd_c - depth / 2), node.box_w, depth,
                    boxstyle='round,pad=0.01,rounding_size=0.014',
                    lw=2.6, edgecolor=C_FRONT_BOX, facecolor=C_FRONT_BOX + '66', zorder=9
                )
                ax_lidar.add_patch(rect)
                box_scan_patches.append(rect)

            # FRENTE
            front_card = artists['front_card']
            if node.box_frente:
                front_card['value'].set_text(_fmt_cm(node.box_frente_dist))
                front_card['status'].set_text(f'CAJA DETECTADA · ancho {node.box_w*100:.0f} cm')
                _set_card_visual(front_card, C_FRONT_BOX)
                _set_card_progress(front_card, min(node.box_frente_dist / 0.50, 1.0))
                artists['front_wedge'].set_facecolor(C_FRONT_BOX)
                artists['front_wedge'].set_edgecolor(C_FRONT_BOX)
                artists['front_wedge'].set_alpha(0.18)
                alert_txt.set_text(f'⚠ CAJA FRONTAL: {node.box_frente_dist*100:.0f} cm · ancho {node.box_w*100:.0f} cm')
                alert_txt.set_color(C_FRONT_BOX)
                box_front_txt.set_text('OBJETO PERPENDICULAR DETECTADO')
                box_front_txt.set_color(C_FRONT_BOX)
            else:
                front_card['value'].set_text(_fmt_cm(node.d_front))
                front_card['status'].set_text('LIBRE' if math.isfinite(node.d_front) else 'sin lectura frontal')
                _set_card_visual(front_card, C_FRONT_OK)
                _set_card_progress(front_card, min((node.d_front if math.isfinite(node.d_front) else 0.0) / 0.70, 1.0))
                artists['front_wedge'].set_facecolor(C_FRONT_OK)
                artists['front_wedge'].set_edgecolor(C_FRONT_OK)
                artists['front_wedge'].set_alpha(0.08)
                alert_txt.set_text('')
                box_front_txt.set_text('')

            # DERECHA
            der_card = artists['der_card']
            dr = node.d_right
            if math.isfinite(dr) and dr <= MAX_GAUGE:
                err = dr - TARGET_DER
                der_card['value'].set_text(_fmt_cm(dr))
                _set_card_progress(der_card, min(dr / MAX_GAUGE, 1.0))
                if abs(err) <= TOL_OK:
                    col = C_OK
                    msg = f'EN OBJETIVO ({err*100:+.0f} cm)'
                elif abs(err) <= 0.08:
                    col = C_WARN
                    msg = f'{"LEJOS" if err > 0 else "CERCA"} · error {abs(err)*100:.0f} cm'
                else:
                    col = C_ALERT
                    msg = f'{"MUY LEJOS" if err > 0 else "MUY CERCA"} · error {abs(err)*100:.0f} cm'
                der_card['status'].set_text(msg)
                _set_card_visual(der_card, col)
            else:
                der_card['value'].set_text('---')
                der_card['status'].set_text('pared no visible')
                _set_card_visual(der_card, C_DIM)
                _set_card_progress(der_card, 0.0)

            # IZQUIERDA + COMANDOS
            dl = node.d_left
            if math.isfinite(dl):
                artists['izq_value'].set_text(_fmt_cm(dl))
                if dl < DIST_IZQ_MIN:
                    izq_col = C_ALERT
                    izq_msg = f'REPULSIÓN · {dl*100:.1f} cm'
                elif dl < DIST_IZQ_WARN:
                    izq_col = C_WARN
                    izq_msg = f'CERCA DEL LÍMITE · {dl*100:.1f} cm'
                else:
                    izq_col = C_LEFT
                    izq_msg = 'OK'
                artists['izq_value'].set_color(izq_col)
                artists['izq_status'].set_text(izq_msg)
                artists['izq_status'].set_color(izq_col)
            else:
                artists['izq_value'].set_text('---')
                artists['izq_value'].set_color(C_DIM)
                artists['izq_status'].set_text('pared no visible')
                artists['izq_status'].set_color(C_DIM)

            cmd_card = artists['cmd_card']
            cmd_card['v_lin'].set_text(f'v: {node.vel_lin:+.3f} m/s')
            cmd_card['v_ang'].set_text(f'ω: {node.vel_ang:+.3f} rad/s')
            cmd_card['corr'].set_text(f'corr lateral: {node.lat_corr:+.3f}')
            if abs(node.vel_ang) > 0.35:
                artists['info_card'].set_edgecolor(C_WARN)
                cmd_card['v_ang'].set_color(C_WARN)
            else:
                artists['info_card'].set_edgecolor(BORDER)
                cmd_card['v_ang'].set_color(C_DIM)

            # Velocidad
            if len(node.vel_times) > 1:
                t_now = time.time() - node._t0
                ts = [t - t_now + MAX_VEL_T for t in node.vel_times]
                vel_line.set_data(ts, list(node.vel_vals))
                ax_vel.set_xlim(0, MAX_VEL_T)
            else:
                vel_line.set_data([], [])

            # FSM
            st = node.fsm_state.strip() if node.fsm_state else '---'
            if st == 'GIRO':
                sc, bg_col = C_WARN, '#1e170b'
            elif st == 'RODEO':
                sc, bg_col = C_RODEO, '#181025'
            else:
                sc, bg_col = C_OK, PANEL

            state_badge.set_text(st)
            state_badge.set_color(sc)
            state_badge.get_bbox_patch().set_edgecolor(sc)
            artists['header_state'].set_text(st)
            artists['header_state'].set_color(sc)
            artists['header_state'].get_bbox_patch().set_edgecolor(sc)

            if node.box_frente:
                ax_lidar.set_facecolor('#1d160d')
            else:
                ax_lidar.set_facecolor(bg_col)

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
