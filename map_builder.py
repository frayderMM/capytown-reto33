#!/usr/bin/env python3
"""
map_builder.py  —  Mapa de ocupacion en tiempo real para CapyTown RC3

Suscribe a /scan + /odom_raw + /cmd_vel + /fsm_state + /cajas_avistadas.
Muestra el mapa a la izquierda y un panel de telemetria a la derecha.

Pensado para la logica simple CRUCERO/GIRO de behavior_fsm.py:
  CRUCERO → pegado a la pared derecha.
  GIRO    → giro izquierdo fijo (amplio) hasta que la pared derecha
             reaparece dentro de dist_pared_lateral.

Uso:
    python3 /root/yahboomcar_ws/src/capytown-reto33/map_builder.py

Guarda al cerrar:
    ~/mapa_capytown.png   imagen del mapa
    ~/mapa_capytown.npy   array numpy

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import math
import os
import threading

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Wedge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import String


# ── Parámetros del mapa ───────────────────────────────────────────────────────
MAP_W  = 3.00
MAP_H  = 1.80
RES    = 0.02
COLS   = int(MAP_W / RES)   # 150
ROWS   = int(MAP_H / RES)   # 90
ORIG_X = 0.30
ORIG_Y = 1.50

LIDAR_FRONT_RAD = math.pi   # 180° = Yahboom MS200
TOPIC_ODOM      = '/odom_raw'

CEL_DESCONOCIDO = 128
CEL_LIBRE       = 240
CEL_OCUPADO     =  20
RAY_STEP        = 3

D_ALERTA    = 0.55   # empieza a frenar
D_OBST      = 0.40   # dispara GIRO
D_PARED_LAT = 0.55   # umbral para volver a CRUCERO (pared der. detectada)
CORREDOR_W  = 0.60

COLOR_TRAJ = {'CRUCERO': 'lime', 'GIRO': 'red'}
COLOR_FSM  = {
    'CRUCERO': ('green', '#E8FFE8'),
    'GIRO':    ('red',   '#FFE8E8'),
}


# ── Utilidades ────────────────────────────────────────────────────────────────
def quat_a_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def mundo_a_celda(xm, ym):
    col = int((xm + ORIG_X) / RES)
    row = int((-ym + ORIG_Y) / RES)
    return (max(0, min(COLS - 1, col)),
            max(0, min(ROWS - 1, row)))


def bresenham(c0, r0, c1, r1):
    cells = []
    dc = abs(c1 - c0); dr = abs(r1 - r0)
    sc = 1 if c1 > c0 else -1
    sr = 1 if r1 > r0 else -1
    err = dc - dr
    c, r = c0, r0
    while True:
        cells.append((c, r))
        if c == c1 and r == r1:
            break
        e2 = 2 * err
        if e2 > -dr: err -= dr; c += sc
        if e2 <  dc: err += dc; r += sr
    return cells


def get_corredor(rx, ry):
    if  0.90 <= ry <= 1.50 and 0.30 <= rx <= 2.10: return 'NORTE'
    if -0.30 <= ry <= 0.30 and 0.30 <= rx <= 2.10: return 'SUR'
    if  2.10 <= rx <= 2.70:                         return 'ESTE'
    if -0.30 <= rx <= 0.30:                         return 'OESTE'
    return '?'


# ── Nodo ROS ──────────────────────────────────────────────────────────────────
class MapBuilder(Node):

    def __init__(self):
        super().__init__('map_builder')
        self.grid        = np.full((ROWS, COLS), CEL_DESCONOCIDO, dtype=np.uint8)
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_yaw   = 0.0
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self.dist_frente = float('inf')
        self.fsm_state   = 'CRUCERO'
        self.vel_v       = 0.0
        self.vel_w       = 0.0
        self.box_count   = 0
        self._lock       = threading.Lock()

        self._lat_lo     = math.radians(60.0)
        self._lat_hi     = math.radians(120.0)
        self._sector_f   = math.radians(45.0)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan,  '/scan',             self._cb_scan, qos)
        self.create_subscription(Odometry,   TOPIC_ODOM,          self._cb_odom, qos)
        self.create_subscription(Twist,      '/cmd_vel',          self._cb_cmd,  10)
        self.create_subscription(String,     '/fsm_state',        self._cb_fsm,  10)
        self.create_subscription(PoseArray,  '/cajas_avistadas',  self._cb_box,  10)

        self.get_logger().info(
            f'map_builder listo | {COLS}x{ROWS} celdas | panel lateral activo')

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            self.robot_x   = msg.pose.pose.position.x
            self.robot_y   = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.robot_yaw = quat_a_yaw(q.x, q.y, q.z, q.w)

    def _cb_cmd(self, msg: Twist):
        with self._lock:
            self.vel_v = msg.linear.x
            self.vel_w = msg.angular.z

    def _cb_fsm(self, msg: String):
        with self._lock:
            self.fsm_state = msg.data

    def _cb_box(self, msg: PoseArray):
        with self._lock:
            self.box_count = len(msg.poses)

    def _cb_scan(self, msg: LaserScan):
        with self._lock:
            rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

        rc, rr = mundo_a_celda(rx, ry)
        d_izq = d_der = d_f = float('inf')

        for i in range(0, len(msg.ranges), RAY_STEP):
            r     = msg.ranges[i]
            valid = math.isfinite(r) and msg.range_min <= r <= msg.range_max

            raw_ang = msg.angle_min + i * msg.angle_increment
            af      = math.atan2(math.sin(raw_ang - LIDAR_FRONT_RAD),
                                  math.cos(raw_ang - LIDAR_FRONT_RAD))
            abs_af  = abs(af)

            if valid:
                if abs_af <= self._sector_f:
                    d_f = min(d_f, r)
                if self._lat_lo <= abs_af <= self._lat_hi:
                    if af > 0: d_izq = min(d_izq, r)
                    else:      d_der = min(d_der, r)
            else:
                continue

            # Mapa de ocupación via Bresenham
            world_ang = ryaw + (raw_ang - LIDAR_FRONT_RAD)
            ex = rx + r * math.cos(world_ang)
            ey = ry + r * math.sin(world_ang)
            ec, er = mundo_a_celda(ex, ey)

            for (c, rw) in bresenham(rc, rr, ec, er)[:-1]:
                if 0 <= rw < ROWS and 0 <= c < COLS:
                    self.grid[rw, c] = CEL_LIBRE
            if 0 <= er < ROWS and 0 <= ec < COLS:
                self.grid[er, ec] = CEL_OCUPADO

        with self._lock:
            self.dist_izq    = d_izq
            self.dist_der    = d_der
            self.dist_frente = d_f

    def snapshot(self):
        with self._lock:
            return (
                self.grid.copy(),
                (self.robot_x, self.robot_y, self.robot_yaw),
                (self.dist_izq, self.dist_der, self.dist_frente),
                (self.vel_v, self.vel_w),
                self.fsm_state,
                self.box_count,
            )


# ── Panel lateral ─────────────────────────────────────────────────────────────
def draw_panel(ax2, fsm, vel_v, vel_w, d_izq, d_der, d_f, box_count, rx, ry):
    ax2.clear()
    ax2.axis('off')
    ax2.set_xlim(0, 1)
    ax2.set_ylim(0, 1)

    col_fsm, bg_fsm = COLOR_FSM.get(fsm, ('gray', '#F0F0F0'))
    y = 0.97

    def txt(s, dy=0.055, fs=10, color='black', bold=False, bg=None, x=0.5, ha='center'):
        nonlocal y
        kw = dict(ha=ha, va='top', fontsize=fs, color=color,
                  fontweight='bold' if bold else 'normal', clip_on=False)
        if bg:
            kw['bbox'] = dict(boxstyle='round,pad=0.35', fc=bg, ec=color, lw=1.5)
        ax2.text(x, y, s, **kw)
        y -= dy

    def sep():
        nonlocal y
        ax2.plot([0.03, 0.97], [y + 0.01, y + 0.01], color='#DDDDDD', lw=0.8)
        y -= 0.02

    # ── Estado FSM ────────────────────────────────────────────────────────────
    txt('ESTADO FSM', dy=0.032, fs=8, color='#888888')
    txt(fsm, dy=0.085, fs=18, color=col_fsm, bold=True, bg=bg_fsm)
    sep()

    # ── Velocidades ───────────────────────────────────────────────────────────
    txt('VELOCIDAD', dy=0.028, fs=8, color='#888888')
    cv = 'green' if vel_v > 0.01 else 'gray'
    cw = '#AA00FF' if abs(vel_w) > 0.05 else 'gray'
    txt(f'v = {vel_v:+.3f} m/s',   dy=0.046, fs=10, color=cv)
    txt(f'ω = {vel_w:+.3f} rad/s', dy=0.062, fs=10, color=cw)
    sep()

    # ── Centrado lateral ──────────────────────────────────────────────────────
    txt('CENTRADO LATERAL', dy=0.028, fs=8, color='#888888')
    BAR_W = 0.58
    for label, dist, c in [('IZQ', d_izq, '#AA00FF'), ('DER', d_der, '#FF8800')]:
        fill    = min(dist, CORREDOR_W) / CORREDOR_W if math.isfinite(dist) else 1.0
        ds      = f'{dist:.2f}m' if math.isfinite(dist) else '--'
        ax2.text(0.04, y, label, va='top', fontsize=8, color=c, fontweight='bold')
        ax2.add_patch(plt.Rectangle((0.20, y - 0.024), BAR_W, 0.022,
                                    fc='#EEEEEE', ec='none', zorder=1))
        ax2.add_patch(plt.Rectangle((0.20, y - 0.024), BAR_W * fill, 0.022,
                                    fc=c, ec='none', alpha=0.75, zorder=2))
        ax2.plot([0.20 + BAR_W * 0.5] * 2, [y - 0.028, y + 0.002],
                 color='black', lw=1.5, zorder=3)
        ax2.text(0.80, y, ds, va='top', fontsize=8, color=c, ha='left')
        y -= 0.048

    if math.isfinite(d_izq) and math.isfinite(d_der):
        err = d_izq - CORREDOR_W / 2.0
        ce  = 'green' if abs(err) < 0.03 else ('#AA00FF' if err > 0 else '#FF8800')
        lab = 'OK' if abs(err) < 0.03 else ('← isla' if err > 0 else '→ pared')
        txt(f'Error: {err*100:+.0f} cm  {lab}', dy=0.060, fs=9, color=ce, bold=True)
    sep()

    # ── Distancia frontal ─────────────────────────────────────────────────────
    txt('DISTANCIA FRONTAL', dy=0.028, fs=8, color='#888888')
    if math.isfinite(d_f):
        cf   = 'red' if d_f < D_OBST else ('darkorange' if d_f < D_ALERTA else 'green')
        zona = 'GIRO !' if d_f < D_OBST else ('ALERTA' if d_f < D_ALERTA else 'LIBRE')
        txt(f'{d_f:.2f} m  [{zona}]', dy=0.065, fs=11, color=cf, bold=True)
    else:
        txt('> rango  [LIBRE]', dy=0.065, fs=11, color='green')
    sep()

    # ── Pared derecha (condición de salida de GIRO) ────────────────────────────
    txt('PARED DER. (umbral GIRO→CRUCERO)', dy=0.028, fs=8, color='#888888')
    if math.isfinite(d_der):
        ok = d_der < D_PARED_LAT
        cg = 'green' if ok else 'gray'
        msg = f'{d_der:.2f} m  {"[DETECTADA]" if ok else "[no visible]"}'
    else:
        cg, msg = 'gray', '--  [no visible]'
    txt(msg, dy=0.065, fs=11, color=cg, bold=True)
    sep()

    # ── Corredor y cajas ──────────────────────────────────────────────────────
    txt('POSICIÓN EN PISTA', dy=0.028, fs=8, color='#888888')
    txt(f'Corredor: {get_corredor(rx, ry)}', dy=0.048, fs=10, color='steelblue', bold=True)
    cbox = 'darkgreen' if box_count >= 5 else 'black'
    txt(f'Cajas: {box_count} / 5', dy=0.05, fs=10, color=cbox)


# ── Guardar mapa ──────────────────────────────────────────────────────────────
def guardar_mapa(grid):
    png = os.path.expanduser('~/mapa_capytown.png')
    npy = os.path.expanduser('~/mapa_capytown.npy')
    plt.imsave(png, grid, cmap='gray', vmin=0, vmax=255)
    np.save(npy, grid)
    print(f'[map_builder] Mapa guardado: {png}')


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = MapBuilder()

    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()

    # ── Figura: mapa (izq) + panel (der) ─────────────────────────────────────
    plt.ion()
    fig = plt.figure(figsize=(14, 7))
    fig.canvas.manager.set_window_title('CapyTown RC3 — Mapa en tiempo real')
    gs  = fig.add_gridspec(1, 2, width_ratios=[2.2, 1.0], wspace=0.06,
                            left=0.04, right=0.98, top=0.93, bottom=0.06)
    ax  = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])

    # Mapa de fondo
    img_plot = ax.imshow(
        node.grid, cmap='gray', vmin=0, vmax=255,
        origin='upper', extent=[0, MAP_W, MAP_H, 0], interpolation='nearest'
    )
    ax.add_patch(mpatches.Rectangle(
        (0, 0), MAP_W, MAP_H, lw=2, ec='cyan', fc='none', ls='--'))

    ax.set_xlim(0, MAP_W); ax.set_ylim(MAP_H, 0)
    ax.set_xlabel('X (m) — Este')
    ax.set_ylabel('Y (m) — Sur')
    ax.set_title('Mapa de ocupación en tiempo real')
    ax.grid(True, alpha=0.2, ls=':')

    leyenda = [
        mpatches.Patch(color='white',   label='Libre'),
        mpatches.Patch(color='black',   label='Pared'),
        mpatches.Patch(color='gray',    label='Desconocido'),
        plt.Line2D([0],[0], color='#AA00FF', lw=2,   label='IZQ'),
        plt.Line2D([0],[0], color='#FF8800', lw=2,   label='DER'),
        plt.Line2D([0],[0], color='lime',    lw=1.5, label='CRUCERO'),
        plt.Line2D([0],[0], color='red',     lw=1.5, label='GIRO'),
    ]
    ax.legend(handles=leyenda, loc='lower right', fontsize=7, ncol=2)

    # Artistas persistentes del mapa
    robot_dot,    = ax.plot([], [], 'ko', ms=10, zorder=10)
    lat_izq_line, = ax.plot([], [], color='#AA00FF', lw=2, alpha=0.85, zorder=6)
    lat_der_line, = ax.plot([], [], color='#FF8800', lw=2, alpha=0.85, zorder=6)

    # Artistas dinámicos (se recrean cada tick)
    dyn_arts = []

    # Trayectoria coloreada por estado
    traj_segs  = []   # [(xs, ys, estado)]  — segmentos completados
    cur_xs, cur_ys, cur_state = [], [], 'CRUCERO'

    def on_close(event):
        grid, *_ = node.snapshot()
        guardar_mapa(grid)

    fig.canvas.mpl_connect('close_event', on_close)

    # ── Loop de visualización ─────────────────────────────────────────────────
    try:
        while rclpy.ok():
            grid, (rx, ry, ryaw), (d_izq, d_der, d_f), \
                (vel_v, vel_w), fsm, box_count = node.snapshot()

            img_plot.set_data(grid)

            mx = rx + ORIG_X
            my = -ry + ORIG_Y

            robot_dot.set_data([mx], [my])

            # Líneas laterales
            ang_izq = ryaw + math.pi * 0.5
            ang_der = ryaw - math.pi * 0.5
            if math.isfinite(d_izq) and d_izq < 1.5:
                lat_izq_line.set_data([mx, mx + d_izq * math.cos(ang_izq)],
                                      [my, my - d_izq * math.sin(ang_izq)])
            else:
                lat_izq_line.set_data([], [])
            if math.isfinite(d_der) and d_der < 1.5:
                lat_der_line.set_data([mx, mx + d_der * math.cos(ang_der)],
                                      [my, my - d_der * math.sin(ang_der)])
            else:
                lat_der_line.set_data([], [])

            # Limpiar artistas dinámicos anteriores
            for a in dyn_arts:
                try: a.remove()
                except Exception: pass
            dyn_arts.clear()

            # Zonas de detección frontal (cuñas semitransparentes)
            map_yaw_deg = -math.degrees(ryaw)
            for radius, color, alpha in [(D_ALERTA, 'yellow', 0.15),
                                          (D_OBST,   'red',    0.25)]:
                w = Wedge((mx, my), radius,
                          map_yaw_deg - 45, map_yaw_deg + 45,
                          fc=color, ec=color, alpha=alpha, zorder=3)
                ax.add_patch(w)
                dyn_arts.append(w)

            # Flecha de heading del robot
            hl = 0.09
            arr_h = ax.annotate(
                '', xy=(mx + hl * math.cos(ryaw), my - hl * math.sin(ryaw)),
                xytext=(mx, my),
                arrowprops=dict(arrowstyle='-|>', color='red', lw=2.5), zorder=12)
            dyn_arts.append(arr_h)

            # Trayectoria coloreada por estado FSM
            if fsm != cur_state and cur_xs:
                cur_xs.append(mx); cur_ys.append(my)
                traj_segs.append((list(cur_xs), list(cur_ys), cur_state))
                cur_xs[:] = [mx]; cur_ys[:] = [my]; cur_state = fsm
            else:
                cur_xs.append(mx); cur_ys.append(my)
                cur_state = fsm

            for xs, ys, st in traj_segs + [(cur_xs, cur_ys, cur_state)]:
                color = COLOR_TRAJ.get(st, 'gray')
                l, = ax.plot(xs, ys, '-', color=color, lw=1.5, alpha=0.7, zorder=4)
                dyn_arts.append(l)

            # Panel lateral
            draw_panel(ax2, fsm, vel_v, vel_w, d_izq, d_der, d_f, box_count, rx, ry)

            fig.canvas.draw_idle()
            plt.pause(0.3)

    except KeyboardInterrupt:
        pass
    finally:
        grid, *_ = node.snapshot()
        guardar_mapa(grid)
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.close('all')


if __name__ == '__main__':
    main()
