#!/usr/bin/env python3
"""
lidar_viz.py — Monitor visual en tiempo real del robot CapyTown.

Ventana matplotlib con:
  [izquierda] Mapa LiDAR
    - Nube de puntos coloreada por sector (frente/izq/der)
    - Cajas detectadas como cuadrados naranjas
    - Rastro de trayectoria (ultimas posiciones por odom)
    - Circulo de rango del sensor
    - Flecha de avance (siempre arriba)
    - Alerta visual si el frente esta muy cerca

  [derecha-arriba] Panel de estado
    - Estado FSM con color segun el estado
    - Distancias frente/izq/der con alerta de color
    - Velocidad lineal y angular
    - Correccion lateral del wall_follower
    - Contador de cajas del censo

  [derecha-abajo] Historial de velocidad (ultimos 10s)

Uso:
  python3 lidar_viz.py               # frente en 180 grados (Yahboom MS200 por defecto)
  python3 lidar_viz.py --front 0     # frente en 0 grados
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
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
import numpy as np


# ── Paleta de colores ──────────────────────────────────────────────────────────
BG      = '#0d1117'
PANEL   = '#161b22'
BORDER  = '#30363d'

C_FRONT  = '#e94560'   # rojo
C_LEFT   = '#4f8ef7'   # azul
C_RIGHT  = '#2ecc71'   # verde
C_OTHER  = '#3a4a5a'   # gris oscuro
C_BOX    = '#f39c12'   # naranja (cajas)
C_TRAJ   = '#8b949e'   # gris claro (trayectoria)
C_ARROW  = '#f5c518'   # amarillo (flecha frente)
C_TEXT   = '#e6edf3'
C_DIM    = '#8b949e'
C_WARN   = '#f39c12'
C_ALERT  = '#e94560'
C_OK     = '#2ecc71'

FSM_COLORS = {
    'CRUCERO': '#2ecc71',
    'PARAR':   '#e94560',
    'RODEAR':  '#9b59b6',
}

# ── Sectores ──────────────────────────────────────────────────────────────────
FRONT_HALF = math.radians(45.0)
SIDE_LO    = math.radians(60.0)
SIDE_HI    = math.radians(120.0)

DIST_WARN  = 0.50   # m — texto naranja
DIST_ALERT = 0.30   # m — texto rojo

MAX_TRAJ   = 400    # puntos de trayectoria a conservar
MAX_VEL_T  = 10.0   # segundos de historial de velocidad


# ── Nodo ROS ──────────────────────────────────────────────────────────────────
class LidarViz(Node):
    def __init__(self, front_deg: float):
        super().__init__('lidar_viz')
        self.front_rad = math.radians(front_deg)

        # Datos LiDAR
        self.scan_pts   = []          # [(x_disp, y_disp, color), ...]
        self.d_front    = float('inf')
        self.d_left     = float('inf')
        self.d_right    = float('inf')
        self.range_min  = 0.0
        self.range_max  = 8.0
        self.n_pts      = 0

        # Velocidad
        self.vel_lin    = 0.0
        self.vel_ang    = 0.0

        # Correccion lateral
        self.lat_corr   = 0.0

        # Estado FSM
        self.fsm_state  = 'ESPERANDO...'

        # Cajas (en odom frame)
        self.cajas_odom = []          # [(x, y), ...]

        # Odometria
        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.robot_yaw  = 0.0
        self.traj_odom  = deque(maxlen=MAX_TRAJ)  # (x, y) en odom

        # Historial velocidad
        self.vel_times  = deque()
        self.vel_vals   = deque()
        self._t0        = time.time()

        # Suscripciones
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan',              self._cb_scan,  qos)
        self.create_subscription(Twist,     '/cmd_vel',           self._cb_cmd,   10)
        self.create_subscription(Float32,   '/lateral_correction',self._cb_lat,   10)
        self.create_subscription(String,    '/fsm_state',         self._cb_fsm,   10)
        self.create_subscription(PoseArray, '/cajas_avistadas',   self._cb_cajas, 10)
        self.create_subscription(Odometry,  '/odom',              self._cb_odom,  10)

    # ── Transformaciones ──────────────────────────────────────────────────────
    def _sensor_to_display(self, theta: float, r: float):
        """Punto del LiDAR (r, theta) → coordenadas de display (frente=arriba)."""
        phi = self.front_rad - theta
        return r * math.sin(phi), r * math.cos(phi)

    def _base_to_display(self, x_b: float, y_b: float):
        """Punto en frame del robot (x=adelante, y=izq) → display."""
        return -y_b, x_b

    def _odom_to_display(self, ox: float, oy: float):
        """Punto en frame odom → display (relativo al robot actual)."""
        dx = ox - self.robot_x
        dy = oy - self.robot_y
        c  = math.cos(-self.robot_yaw)
        s  = math.sin(-self.robot_yaw)
        x_b =  c * dx - s * dy   # en frame robot
        y_b =  s * dx + c * dy
        return self._base_to_display(x_b, y_b)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _cb_scan(self, msg: LaserScan):
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        pts = []
        d_f = d_l = d_r = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r == 0.0:
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            theta  = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(theta - self.front_rad),
                                math.cos(theta - self.front_rad))
            abs_af = abs(af)

            if abs_af <= FRONT_HALF:
                color = C_FRONT;  d_f = min(d_f, r)
            elif SIDE_LO <= abs_af <= SIDE_HI:
                if af > 0:
                    color = C_LEFT;  d_l = min(d_l, r)
                else:
                    color = C_RIGHT; d_r = min(d_r, r)
            else:
                color = C_OTHER

            xd, yd = self._sensor_to_display(theta, r)
            pts.append((xd, yd, color))

        self.scan_pts = pts
        self.d_front  = d_f
        self.d_left   = d_l
        self.d_right  = d_r
        self.n_pts    = len(pts)

    def _cb_cmd(self, msg: Twist):
        self.vel_lin = msg.linear.x
        self.vel_ang = msg.angular.z
        t = time.time() - self._t0
        self.vel_times.append(t)
        self.vel_vals.append(msg.linear.x)
        # Purgar datos mas viejos que MAX_VEL_T segundos
        while self.vel_times and (t - self.vel_times[0]) > MAX_VEL_T:
            self.vel_times.popleft()
            self.vel_vals.popleft()

    def _cb_lat(self, msg: Float32):
        self.lat_corr = msg.data

    def _cb_fsm(self, msg: String):
        self.fsm_state = msg.data

    def _cb_cajas(self, msg: PoseArray):
        self.cajas_odom = [(p.position.x, p.position.y) for p in msg.poses]

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x   = p.x
        self.robot_y   = p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        self.robot_yaw = math.atan2(siny, cosy)
        self.traj_odom.append((p.x, p.y))


# ── Construccion de la figura ─────────────────────────────────────────────────
def build_figure():
    fig = plt.figure(figsize=(14, 8), facecolor=BG)
    gs  = gridspec.GridSpec(2, 2,
                            width_ratios=[2.2, 1],
                            height_ratios=[2.5, 1],
                            hspace=0.35, wspace=0.28,
                            left=0.05, right=0.97,
                            top=0.95, bottom=0.07)

    ax_lidar = fig.add_subplot(gs[:, 0])   # izquierda, toda la altura
    ax_info  = fig.add_subplot(gs[0, 1])   # derecha arriba
    ax_vel   = fig.add_subplot(gs[1, 1])   # derecha abajo

    # ── Mapa LiDAR ────────────────────────────────────────────────────────
    for ax in [ax_lidar, ax_info, ax_vel]:
        ax.set_facecolor(PANEL)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)
        ax.tick_params(colors=C_DIM, labelsize=8)

    ax_lidar.set_xlim(-4.5, 4.5)
    ax_lidar.set_ylim(-4.5, 4.5)
    ax_lidar.set_aspect('equal')
    ax_lidar.grid(True, color=BORDER, linewidth=0.5, linestyle='--')
    ax_lidar.set_title('LiDAR — Vista en tiempo real', color=C_TEXT,
                       fontsize=12, pad=8)
    ax_lidar.set_xlabel('← IZQ   DER →', color=C_DIM, fontsize=8)
    ax_lidar.set_ylabel('↑ FRENTE', color=C_DIM, fontsize=8)

    # Circulos de referencia
    for d in [1, 2, 3, 4]:
        ax_lidar.add_patch(plt.Circle((0, 0), d, color=BORDER,
                                      fill=False, lw=0.7, ls=':'))
        ax_lidar.text(0.05, d + 0.08, f'{d}m', color=BORDER, fontsize=7)

    # Robot en el centro
    ax_lidar.scatter([0], [0], s=80, c='white', zorder=6)

    # Flecha frente (siempre arriba)
    ax_lidar.annotate('', xy=(0, 0.7), xytext=(0, 0),
                      arrowprops=dict(arrowstyle='->', color=C_ARROW, lw=2.5),
                      zorder=7)

    # Scatter LiDAR
    scatter = ax_lidar.scatter([], [], s=3, zorder=4)

    # Trayectoria
    traj_line, = ax_lidar.plot([], [], color=C_TRAJ, lw=1.2,
                               alpha=0.6, zorder=3)

    # Cajas (lista de parches, se recrea cada frame)
    box_patches = []

    # Circulo de rango del sensor
    range_circ = plt.Circle((0, 0), 8.0, color='#223344',
                             fill=False, lw=1, ls='--', zorder=2)
    ax_lidar.add_patch(range_circ)

    # Texto de alerta de distancia (centro-inferior del mapa)
    alert_txt = ax_lidar.text(0, -4.2, '', color=C_ALERT, fontsize=11,
                              ha='center', va='bottom', fontweight='bold', zorder=8)

    # Leyenda
    ax_lidar.legend(handles=[
        mpatches.Patch(color=C_FRONT, label='FRENTE'),
        mpatches.Patch(color=C_LEFT,  label='IZQ'),
        mpatches.Patch(color=C_RIGHT, label='DER'),
        mpatches.Patch(color=C_BOX,   label='Caja'),
    ], loc='upper right', facecolor='#0d1117', labelcolor=C_TEXT,
       fontsize=8, framealpha=0.9)

    # ── Panel de info ─────────────────────────────────────────────────────
    ax_info.set_xlim(0, 1)
    ax_info.set_ylim(0, 1)
    ax_info.axis('off')
    ax_info.set_title('Estado', color=C_TEXT, fontsize=11, pad=6)

    fsm_box = ax_info.text(0.5, 0.92, 'ESPERANDO...', ha='center', va='top',
                           color='white', fontsize=14, fontweight='bold',
                           transform=ax_info.transAxes,
                           bbox=dict(boxstyle='round,pad=0.4',
                                     facecolor='#333', edgecolor=BORDER))

    info_txt = ax_info.text(0.05, 0.70, '', ha='left', va='top',
                            color=C_TEXT, fontsize=9.5, fontfamily='monospace',
                            transform=ax_info.transAxes,
                            linespacing=1.8)

    # ── Grafico velocidad ──────────────────────────────────────────────────
    ax_vel.set_title('Velocidad lineal (m/s)', color=C_TEXT, fontsize=10, pad=5)
    ax_vel.set_facecolor(PANEL)
    ax_vel.set_xlim(0, MAX_VEL_T)
    ax_vel.set_ylim(-0.25, 0.35)
    ax_vel.axhline(0, color=BORDER, lw=0.8)
    ax_vel.set_xlabel('Tiempo (s)', color=C_DIM, fontsize=8)
    ax_vel.set_ylabel('m/s', color=C_DIM, fontsize=8)
    vel_line, = ax_vel.plot([], [], color=C_FRONT, lw=1.5)

    return (fig, ax_lidar, ax_info, ax_vel,
            scatter, traj_line, box_patches, range_circ, alert_txt,
            fsm_box, info_txt, vel_line)


# ── Helpers ───────────────────────────────────────────────────────────────────
def dist_str(v):
    return f'{v:.2f} m' if math.isfinite(v) else '  ---  '

def dist_color(v):
    if v <= DIST_ALERT: return C_ALERT
    if v <= DIST_WARN:  return C_WARN
    return C_OK

def vel_label(lin, ang):
    d = 'ADELANTE' if lin >  0.01 else 'ATRAS' if lin < -0.01 else 'PARADO'
    g = 'IZQ'      if ang >  0.01 else 'DER'   if ang < -0.01 else 'RECTO'
    return d, g


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--front', type=float, default=180.0,
                        help='Angulo del frente en el LiDAR (grados). Default: 180')
    args = parser.parse_args()

    rclpy.init()
    node = LidarViz(front_deg=args.front)

    plt.ion()
    (fig, ax_lidar, ax_info, ax_vel,
     scatter, traj_line, box_patches, range_circ, alert_txt,
     fsm_box, info_txt, vel_line) = build_figure()
    plt.show()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.04)

            # ── Puntos LiDAR ──────────────────────────────────────────────
            if node.scan_pts:
                xs = [p[0] for p in node.scan_pts]
                ys = [p[1] for p in node.scan_pts]
                cs = [p[2] for p in node.scan_pts]
                scatter.set_offsets(np.column_stack([xs, ys]))
                scatter.set_color(cs)

            # ── Trayectoria ───────────────────────────────────────────────
            if len(node.traj_odom) > 1:
                txs = [node._odom_to_display(x, y)[0]
                       for x, y in node.traj_odom]
                tys = [node._odom_to_display(x, y)[1]
                       for x, y in node.traj_odom]
                traj_line.set_data(txs, tys)

            # ── Cajas en el mapa ──────────────────────────────────────────
            for p in box_patches:
                p.remove()
            box_patches.clear()
            for ox, oy in node.cajas_odom:
                xd, yd = node._odom_to_display(ox, oy)
                rect = mpatches.FancyBboxPatch(
                    (xd - 0.10, yd - 0.10), 0.20, 0.20,
                    boxstyle='square,pad=0', linewidth=1.5,
                    edgecolor=C_BOX, facecolor=C_BOX + '44', zorder=5)
                ax_lidar.add_patch(rect)
                box_patches.append(rect)

            # ── Rango del sensor ──────────────────────────────────────────
            range_circ.set_radius(min(node.range_max, 4.4))

            # ── Alerta visual en el mapa ──────────────────────────────────
            if node.d_front <= DIST_ALERT:
                alert_txt.set_text(f'⚠ FRENTE: {node.d_front:.2f}m')
                alert_txt.set_color(C_ALERT)
                ax_lidar.set_facecolor('#1a0a0a')
            elif node.d_front <= DIST_WARN:
                alert_txt.set_text(f'! FRENTE: {node.d_front:.2f}m')
                alert_txt.set_color(C_WARN)
                ax_lidar.set_facecolor('#1a1208')
            else:
                alert_txt.set_text('')
                ax_lidar.set_facecolor(PANEL)

            # ── FSM state box ─────────────────────────────────────────────
            state = node.fsm_state
            fsm_col = FSM_COLORS.get(state, '#555555')
            fsm_box.set_text(state)
            fsm_box.get_bbox_patch().set_facecolor(fsm_col + 'aa')
            fsm_box.get_bbox_patch().set_edgecolor(fsm_col)

            # ── Panel de info ─────────────────────────────────────────────
            d_vel, g_vel = vel_label(node.vel_lin, node.vel_ang)
            lat_bar = '▓' * min(int(abs(node.lat_corr) * 10), 8)
            lat_dir = 'IZQ' if node.lat_corr < -0.005 else \
                      'DER' if node.lat_corr >  0.005 else 'CENT'

            info_txt.set_text(
                f'FRENTE : {dist_str(node.d_front)}\n'
                f'IZQ    : {dist_str(node.d_left)}\n'
                f'DER    : {dist_str(node.d_right)}\n'
                f'\n'
                f'Vel lin : {node.vel_lin:+.3f} m/s  {d_vel}\n'
                f'Vel ang : {node.vel_ang:+.3f} rad/s  {g_vel}\n'
                f'Lat corr: {node.lat_corr:+.3f}  {lat_dir} {lat_bar}\n'
                f'\n'
                f'Cajas censo : {len(node.cajas_odom)}\n'
                f'Puntos scan : {node.n_pts}\n'
                f'Rango sensor: {node.range_min:.1f}–{node.range_max:.1f} m'
            )

            # ── Historial de velocidad ─────────────────────────────────────
            if len(node.vel_times) > 1:
                t_now = time.time() - node._t0
                ts = [t - t_now + MAX_VEL_T for t in node.vel_times]
                vel_line.set_data(ts, list(node.vel_vals))
                ax_vel.set_xlim(0, MAX_VEL_T)

            fig.canvas.draw_idle()
            plt.pause(0.04)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
