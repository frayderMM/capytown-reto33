#!/usr/bin/env python3
"""
lidar_viz.py — Monitor visual en tiempo real del robot CapyTown.

Ventana matplotlib con:
  [izquierda] Mapa LiDAR
    - Nube de puntos coloreada por sector (frente/izq/der)
    - Cajas detectadas (odom) como cuadrados naranjas
    - Caja en FRENTE resaltada (recta tipo T, ancho ~20cm) en rojo
    - Rastro de trayectoria (ultimas posiciones por odom)
    - Circulo de rango del sensor

  [derecha-arriba] Gauge de pared derecha
    - Barra horizontal: 0–70 cm, banda verde centrada en objetivo 8 cm
    - Numero exacto en cm (grande, color segun error)
    - Indicador BOX cuando hay una caja perpendicular al frente

  [derecha-abajo] Historial de velocidad lineal (ultimos 10 s)

Uso:
  python3 lidar_viz.py               # frente en 180° (Yahboom MS200)
  python3 lidar_viz.py --front 0     # frente en 0°
"""

import math
import argparse
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
import numpy as np


# ── Paleta ────────────────────────────────────────────────────────────────────
BG      = '#0d1117'
PANEL   = '#161b22'
BORDER  = '#30363d'
C_FRONT = '#e94560'
C_LEFT  = '#4f8ef7'
C_RIGHT = '#2ecc71'
C_OTHER = '#3a4a5a'
C_BOX   = '#f39c12'
C_TRAJ  = '#8b949e'
C_ARROW = '#f5c518'
C_TEXT  = '#e6edf3'
C_DIM   = '#8b949e'
C_WARN  = '#f39c12'
C_ALERT = '#e94560'
C_OK    = '#2ecc71'

# ── Constantes ────────────────────────────────────────────────────────────────
FRONT_HALF      = math.radians(30.0)   # sector ancho (vel + emergencia)
FRONT_GIRO_HALF = math.radians(12.0)   # sector estrecho (dispara GIRO)
SIDE_LO    = math.radians(60.0)
SIDE_HI    = math.radians(120.0)

TARGET_DER  = 0.08   # m  objetivo de pegamiento a la pared derecha
TOL_OK      = 0.02   # m  +/- tolerancia para considerar "en objetivo"
MAX_GAUGE   = 0.70   # m  maximo del gauge

BOX_W_MIN   = 0.10   # m  ancho minimo para clasificar como caja (no pared)
BOX_W_MAX   = 0.35   # m  ancho maximo (caja CapyTown ~20cm +/- margen)
BOX_DEPTH   = 0.10   # m  profundidad maxima del cluster (face plana)
BOX_MAX_R   = 1.20   # m  solo busca cajas dentro de este rango frontal

DIST_WARN   = 0.50
DIST_ALERT  = 0.30
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

        # Deteccion de caja al frente (recta tipo T)
        self.box_frente      = False   # hay caja perpendicular al frente
        self.box_frente_dist = float('inf')  # distancia a esa caja

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
        self.create_subscription(LaserScan, '/scan',               self._cb_scan,  qos)
        self.create_subscription(Odometry,  '/odom_raw',           self._cb_odom,  qos)
        self.create_subscription(Twist,     '/cmd_vel',            self._cb_cmd,   10)
        self.create_subscription(Float32,   '/lateral_correction', self._cb_lat,   10)
        self.create_subscription(PoseArray, '/cajas_avistadas',    self._cb_cajas, 10)

    # ── Transformaciones ──────────────────────────────────────────────────────
    def _sensor_to_display(self, theta, r):
        phi = self.front_rad - theta
        return r * math.sin(phi), r * math.cos(phi)

    def _odom_to_display(self, ox, oy):
        dx, dy = ox - self.robot_x, oy - self.robot_y
        c = math.cos(-self.robot_yaw); s = math.sin(-self.robot_yaw)
        xb = c * dx - s * dy
        yb = s * dx + c * dy
        return -yb, xb

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _cb_scan(self, msg: LaserScan):
        segs      = []   # [[(x0,y0),(x1,y1)], ...]  segmentos de linea
        seg_cols  = []   # color por segmento
        d_f = d_l = d_r = float('inf')
        front_rf  = []   # (xr, yr) en frame robot, cono estrecho, para deteccion caja
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

            # Solo el cono estrecho ±12° se colorea como FRENTE (blanco).
            # El sector 12°-30° pasa a gris — evita que la pared lateral
            # aparezca como "obstáculo frontal" en el visor.
            if abs_af <= FRONT_GIRO_HALF:
                color = '#ffffff'; d_f = min(d_f, r)
                if r <= BOX_MAX_R:
                    front_rf.append((r * math.cos(af), r * math.sin(af)))
            elif SIDE_LO <= abs_af <= SIDE_HI:
                if af > 0:
                    color = C_LEFT;  d_l = min(d_l, r)
                else:
                    color = C_RIGHT; d_r = min(d_r, r)
            else:
                color = C_OTHER

            xd, yd = self._sensor_to_display(theta, r)

            # Conectar con el punto anterior solo si estan cerca
            # (sin salto de rango grande = misma superficie continua).
            if (prev_xd is not None and prev_r is not None
                    and abs(r - prev_r) < 0.20
                    and math.hypot(xd - prev_xd, yd - prev_yd) < 0.30):
                segs.append([(prev_xd, prev_yd), (xd, yd)])
                seg_cols.append(color)

            prev_xd, prev_yd, prev_r = xd, yd, r

        self.scan_segs     = segs
        self.scan_seg_cols = seg_cols
        self.d_front  = d_f
        self.d_left   = d_l
        self.d_right  = d_r

        # ── Deteccion de caja (recta tipo T al frente) ────────────────────
        # Una caja frontal perpendicular (forma T) tiene:
        #   - cluster de puntos a la misma profundidad (xr constante)
        #   - ancho lateral (yr spread) entre BOX_W_MIN y BOX_W_MAX
        #   - cluster centrado: y_center cerca de 0 (no todo a un lado = pared en angulo)
        # Una pared lateral que entra en el cono en angulo tiene todos sus puntos
        # a un lado del eje (y_center grande) y se filtra por ese chequeo.
        self.box_frente      = False
        self.box_frente_dist = float('inf')
        if len(front_rf) >= 3:
            front_rf.sort(key=lambda p: p[0])   # ordenar por profundidad
            d_min_x = front_rf[0][0]
            # Tomar el cluster mas cercano (dentro de BOX_DEPTH en profundidad)
            cluster = [(x, y) for x, y in front_rf if x <= d_min_x + BOX_DEPTH]
            if len(cluster) >= 3:
                ys       = [p[1] for p in cluster]
                width    = max(ys) - min(ys)
                y_center = (max(ys) + min(ys)) / 2.0
                # Ancho razonable Y cluster centrado (no todo a un lado)
                if BOX_W_MIN <= width <= BOX_W_MAX and abs(y_center) < 0.15:
                    self.box_frente      = True
                    self.box_frente_dist = d_min_x

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


# ── Figura ────────────────────────────────────────────────────────────────────
def build_figure():
    fig = plt.figure(figsize=(11, 6), facecolor=BG)
    gs  = gridspec.GridSpec(2, 2,
                            width_ratios=[2.2, 1],
                            height_ratios=[2.8, 1],
                            hspace=0.38, wspace=0.28,
                            left=0.05, right=0.97,
                            top=0.94, bottom=0.07)

    ax_lidar = fig.add_subplot(gs[:, 0])
    ax_gauge = fig.add_subplot(gs[0, 1])
    ax_vel   = fig.add_subplot(gs[1, 1])

    for ax in [ax_lidar, ax_gauge, ax_vel]:
        ax.set_facecolor(PANEL)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)
        ax.tick_params(colors=C_DIM, labelsize=8)

    # ── LiDAR ─────────────────────────────────────────────────────────────
    ax_lidar.set_xlim(-1.5, 1.5)
    ax_lidar.set_ylim(-1.5, 1.5)
    ax_lidar.set_aspect('equal')
    ax_lidar.grid(True, color=BORDER, lw=0.5, ls='--')
    ax_lidar.set_title('LiDAR — Vista en tiempo real', color=C_TEXT, fontsize=12, pad=8)
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
    alert_txt = ax_lidar.text(0, -1.42, '', color=C_ALERT, fontsize=11,
                              ha='center', va='bottom', fontweight='bold', zorder=8)
    box_front_txt = ax_lidar.text(0, 1.42, '', color=C_BOX, fontsize=10,
                                  ha='center', va='top', fontweight='bold', zorder=9)
    ax_lidar.legend(handles=[
        mpatches.Patch(color='#ffffff', label='FRENTE ±12°'),
        mpatches.Patch(color=C_LEFT,    label='IZQ'),
        mpatches.Patch(color=C_RIGHT,   label='DER'),
        mpatches.Patch(color=C_BOX,     label='Caja'),
    ], loc='upper right', facecolor=BG, labelcolor=C_TEXT, fontsize=8, framealpha=0.9)

    # ── Gauge pared derecha ────────────────────────────────────────────────
    ax_gauge.axis('off')
    ax_gauge.set_xlim(0, 1)
    ax_gauge.set_ylim(0, 1)
    ax_gauge.set_title('PARED DERECHA', color=C_TEXT, fontsize=11, pad=6)
    # Artistas del gauge (se actualizan en el loop)
    gauge_artists = {}   # claves: 'bar_bg', 'bar_fill', 'needle', 'lbl_val', 'lbl_err', 'lbl_box'

    # Fondo del gauge (barra horizontal)
    ax_gauge.add_patch(mpatches.FancyBboxPatch(
        (0.08, 0.66), 0.84, 0.10, boxstyle='round,pad=0.01',
        fc='#1e2a1e', ec=BORDER, lw=1.5, zorder=1))

    # Zona OK: TARGET +/- TOL_OK en verde translucido
    ok_lo = (TARGET_DER - TOL_OK) / MAX_GAUGE
    ok_hi = (TARGET_DER + TOL_OK) / MAX_GAUGE
    ax_gauge.add_patch(mpatches.Rectangle(
        (0.08 + 0.84 * ok_lo, 0.66), 0.84 * (ok_hi - ok_lo), 0.10,
        fc='#1a4a1a', ec='none', zorder=2))

    # Linea de objetivo
    tgt_x = 0.08 + 0.84 * (TARGET_DER / MAX_GAUGE)
    ax_gauge.plot([tgt_x, tgt_x], [0.64, 0.78], color=C_OK, lw=2.0, zorder=4)
    ax_gauge.text(tgt_x, 0.60, f'{int(TARGET_DER*100)} cm\nobjetivo',
                  ha='center', va='top', color=C_OK, fontsize=7)

    # Tick labels del gauge (0, 20, 40, 60, 70 cm)
    for cm in [0, 10, 20, 30, 40, 50, 60, 70]:
        xp = 0.08 + 0.84 * (cm / 100 / MAX_GAUGE)
        if xp > 0.92:
            break
        ax_gauge.text(xp, 0.80, f'{cm}', ha='center', va='bottom',
                      color=C_DIM, fontsize=6.5)
        ax_gauge.plot([xp, xp], [0.77, 0.79], color=BORDER, lw=0.8, zorder=1)

    # Barra rellena (nivel actual) — se actualiza en el loop
    bar_fill = mpatches.FancyBboxPatch(
        (0.08, 0.67), 0.0, 0.08, boxstyle='round,pad=0.0',
        fc=C_OK, ec='none', zorder=3)
    ax_gauge.add_patch(bar_fill)
    gauge_artists['bar_fill'] = bar_fill

    # Marcador vertical actual (needle)
    needle, = ax_gauge.plot([], [], color='white', lw=2.5, zorder=5)
    gauge_artists['needle'] = needle

    # Valor en cm (texto grande)
    lbl_val = ax_gauge.text(0.5, 0.50, '---', ha='center', va='center',
                            color=C_TEXT, fontsize=30, fontweight='bold')
    gauge_artists['lbl_val'] = lbl_val

    # Error respecto al objetivo
    lbl_err = ax_gauge.text(0.5, 0.30, '', ha='center', va='center',
                            color=C_DIM, fontsize=10)
    gauge_artists['lbl_err'] = lbl_err

    # Separador
    ax_gauge.plot([0.05, 0.95], [0.19, 0.19], color=BORDER, lw=0.8)

    # Indicador de caja al frente
    lbl_box = ax_gauge.text(0.5, 0.18, '  SIN CAJA  ', ha='center', va='center',
                            color=C_DIM, fontsize=11, fontweight='bold',
                            bbox=dict(boxstyle='round,pad=0.4', fc='#222', ec=BORDER, lw=1.5))
    gauge_artists['lbl_box'] = lbl_box

    # Pared izquierda (dato secundario)
    ax_gauge.text(0.05, 0.06, 'IZQ:', ha='left', va='center',
                  color=C_DIM, fontsize=8)
    lbl_izq = ax_gauge.text(0.25, 0.06, '---', ha='left', va='center',
                            color=C_LEFT, fontsize=9, fontweight='bold')
    gauge_artists['lbl_izq'] = lbl_izq

    # ── Velocidad ──────────────────────────────────────────────────────────
    ax_vel.set_title('Vel. lineal (m/s)', color=C_TEXT, fontsize=9, pad=4)
    ax_vel.set_xlim(0, MAX_VEL_T)
    ax_vel.set_ylim(-0.25, 0.30)
    ax_vel.axhline(0, color=BORDER, lw=0.8)
    ax_vel.set_xlabel('t (s)', color=C_DIM, fontsize=7)
    vel_line, = ax_vel.plot([], [], color=C_FRONT, lw=1.5)

    return (fig, ax_lidar, ax_gauge, ax_vel,
            lc, traj_line, box_patches, range_circ,
            alert_txt, box_front_txt, gauge_artists, vel_line)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--front', type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = LidarViz(front_deg=args.front)

    plt.ion()
    (fig, ax_lidar, ax_gauge, ax_vel,
     lc, traj_line, box_patches, range_circ,
     alert_txt, box_front_txt, gauge_artists, vel_line) = build_figure()
    plt.show()

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

            # ── Rango sensor ──────────────────────────────────────────────
            range_circ.set_radius(min(node.range_max, 1.4))

            # ── Alerta frontal en LiDAR ───────────────────────────────────
            # Solo dispara si se detecta una linea perpendicular tipo T (caja).
            # La pared lateral que entra en angulo al cono no activa esta alerta.
            if node.box_frente:
                alert_txt.set_text(f'! CAJA PERPENDICULAR: {node.box_frente_dist:.2f} m')
                alert_txt.set_color(C_ALERT)
                ax_lidar.set_facecolor('#1a0a0a')
            else:
                alert_txt.set_text('')
                ax_lidar.set_facecolor(PANEL)

            # ── Indicador de caja al frente en LiDAR ─────────────────────
            if node.box_frente:
                box_front_txt.set_text(f'[CAJA] {node.box_frente_dist*100:.0f} cm')
                box_front_txt.set_color(C_BOX)
            else:
                box_front_txt.set_text('')

            # ── Gauge pared derecha ───────────────────────────────────────
            dr = node.d_right   # metros
            ga = gauge_artists

            if math.isfinite(dr) and dr <= MAX_GAUGE:
                frac  = min(dr / MAX_GAUGE, 1.0)
                err   = dr - TARGET_DER       # positivo = muy lejos, negativo = muy cerca

                if abs(err) <= TOL_OK:
                    col = C_OK
                    err_txt = f'✓ en objetivo  ({err*100:+.0f} cm)'
                elif abs(err) <= 0.08:
                    col = C_WARN
                    err_txt = f'{"alejado" if err > 0 else "pegado"} {abs(err)*100:.0f} cm del objetivo'
                else:
                    col = C_ALERT
                    err_txt = f'{"muy lejos" if err > 0 else "muy cerca"} ({abs(err)*100:.0f} cm del objetivo)'

                # Barra rellena
                ga['bar_fill'].set_x(0.08)
                ga['bar_fill'].set_width(0.84 * frac)
                ga['bar_fill'].set_facecolor(col + 'bb')

                # Aguja
                nx = 0.08 + 0.84 * frac
                ga['needle'].set_data([nx, nx], [0.64, 0.78])

                # Numero grande
                ga['lbl_val'].set_text(f'{dr*100:.1f} cm')
                ga['lbl_val'].set_color(col)

                # Error texto
                ga['lbl_err'].set_text(err_txt)
                ga['lbl_err'].set_color(col)

            else:
                ga['bar_fill'].set_width(0.0)
                ga['needle'].set_data([], [])
                ga['lbl_val'].set_text('---')
                ga['lbl_val'].set_color(C_DIM)
                ga['lbl_err'].set_text('pared derecha no visible')
                ga['lbl_err'].set_color(C_DIM)

            # IZQ (secundario)
            if math.isfinite(node.d_left):
                ga['lbl_izq'].set_text(f'{node.d_left*100:.1f} cm')
            else:
                ga['lbl_izq'].set_text('---')

            # Indicador caja
            if node.box_frente:
                ga['lbl_box'].set_text(f'  CAJA FRENTE: {node.box_frente_dist*100:.0f} cm  ')
                ga['lbl_box'].set_color(C_BOX)
                ga['lbl_box'].get_bbox_patch().set_edgecolor(C_BOX)
                ga['lbl_box'].get_bbox_patch().set_facecolor('#2a1800')
            else:
                ga['lbl_box'].set_text('  SIN CAJA  ')
                ga['lbl_box'].set_color(C_DIM)
                ga['lbl_box'].get_bbox_patch().set_edgecolor(BORDER)
                ga['lbl_box'].get_bbox_patch().set_facecolor('#222')

            # ── Historial velocidad ───────────────────────────────────────
            if len(node.vel_times) > 1:
                t_now = time.time() - node._t0
                ts = [t - t_now + MAX_VEL_T for t in node.vel_times]
                vel_line.set_data(ts, list(node.vel_vals))
                ax_vel.set_xlim(0, MAX_VEL_T)

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
