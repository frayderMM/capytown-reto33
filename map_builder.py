#!/usr/bin/env python3
"""
map_builder.py  —  Mapa de ocupacion en tiempo real para CapyTown RC3

Suscribe a /scan + /odom y construye un mapa 2D de la pista.
Muestra el mapa en pantalla y lo guarda al cerrar.

Pista: 3.00 x 1.80 m  |  Resolucion: 2 cm/celda  ->  150 x 90 celdas

Uso:
    python3 /root/frayder_ws/src/capytown_esan/map_builder.py

Guarda al cerrar la ventana o Ctrl-C:
    ~/mapa_capytown.png   imagen del mapa
    ~/mapa_capytown.npy   array numpy (para analisis posterior)

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import math
import os
import threading
import time

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrow

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


# ── Parametros del mapa ───────────────────────────────────────────────────────
MAP_W   = 3.00   # metros (eje x = este)
MAP_H   = 1.80   # metros (eje y = norte)
RES     = 0.02   # metros por celda (2 cm)
COLS    = int(MAP_W / RES)   # 150
ROWS    = int(MAP_H / RES)   # 90

# Posicion inicial del robot en el mapa (donde esta el (0,0) del odom)
# Robot inicia en esquina inferior-izquierda, corredor sur, centrado
ORIG_X  = 0.30   # m desde la pared oeste al punto de inicio del robot
ORIG_Y  = 1.50   # m desde la pared norte al punto de inicio del robot

LIDAR_FRONT_RAD = math.pi   # 180 deg = Yahboom MS200

# Valores de celda (escala de grises)
CEL_DESCONOCIDO = 128   # gris
CEL_LIBRE       = 240   # blanco
CEL_OCUPADO     =  20   # negro

# Subsampleo de rayos LiDAR (1 de cada N) para no saturar CPU
RAY_STEP = 3


# ── Utilidades ────────────────────────────────────────────────────────────────
def quat_a_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def mundo_a_celda(xm, ym):
    """Coordenadas odom (m) -> (col, row) en el grid."""
    col = int((xm + ORIG_X) / RES)
    row = int((-ym + ORIG_Y) / RES)   # y-norte -> row-sur (invertido)
    return (max(0, min(COLS - 1, col)),
            max(0, min(ROWS - 1, row)))


def bresenham(c0, r0, c1, r1):
    """Celdas del segmento (c0,r0)->(c1,r1)."""
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


# ── Nodo ROS ──────────────────────────────────────────────────────────────────
class MapBuilder(Node):

    def __init__(self):
        super().__init__('map_builder')
        self.grid      = np.full((ROWS, COLS), CEL_DESCONOCIDO, dtype=np.uint8)
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.dist_izq  = float('inf')   # distancia lateral izquierda (m)
        self.dist_der  = float('inf')   # distancia lateral derecha (m)
        self._lock     = threading.Lock()

        # Sectores laterales: 60°–120° a cada lado del frente
        self._lat_lo = math.radians(60.0)
        self._lat_hi = math.radians(120.0)
        self._lidar_front = math.pi   # 180° = Yahboom MS200

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos)
        self.create_subscription(Odometry,  '/odom', self._cb_odom, 10)
        self.get_logger().info(
            f'map_builder listo | {COLS}x{ROWS} celdas | '
            f'{MAP_W:.1f}x{MAP_H:.1f} m | {int(RES*100)} cm/celda')

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            self.robot_x   = msg.pose.pose.position.x
            self.robot_y   = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.robot_yaw = quat_a_yaw(q.x, q.y, q.z, q.w)

    def _cb_scan(self, msg: LaserScan):
        with self._lock:
            rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

        rc, rr = mundo_a_celda(rx, ry)
        d_izq = float('inf')
        d_der = float('inf')

        for i in range(0, len(msg.ranges), RAY_STEP):
            r = msg.ranges[i]
            valid = math.isfinite(r) and msg.range_min <= r <= msg.range_max
            if not valid:
                continue

            raw_ang   = msg.angle_min + i * msg.angle_increment
            af        = math.atan2(math.sin(raw_ang - self._lidar_front),
                                   math.cos(raw_ang - self._lidar_front))
            abs_af    = abs(af)

            # Distancias laterales (sectores 60°–120°)
            if self._lat_lo <= abs_af <= self._lat_hi:
                if af > 0:
                    d_izq = min(d_izq, r)
                else:
                    d_der = min(d_der, r)

            world_ang = ryaw + (raw_ang - LIDAR_FRONT_RAD)
            ex = rx + r * math.cos(world_ang)
            ey = ry + r * math.sin(world_ang)
            ec, er = mundo_a_celda(ex, ey)

            # Celdas libres a lo largo del rayo (sin incluir el extremo)
            for (c, rw) in bresenham(rc, rr, ec, er)[:-1]:
                if 0 <= rw < ROWS and 0 <= c < COLS:
                    self.grid[rw, c] = CEL_LIBRE

            # Extremo = ocupado
            if 0 <= er < ROWS and 0 <= ec < COLS:
                self.grid[er, ec] = CEL_OCUPADO

        with self._lock:
            self.dist_izq = d_izq
            self.dist_der = d_der

    def snapshot(self):
        with self._lock:
            return (self.grid.copy(),
                    (self.robot_x, self.robot_y, self.robot_yaw),
                    (self.dist_izq, self.dist_der))


# ── Visualizacion ─────────────────────────────────────────────────────────────
def guardar_mapa(grid):
    png = os.path.expanduser('~/mapa_capytown.png')
    npy = os.path.expanduser('~/mapa_capytown.npy')
    plt.imsave(png, grid, cmap='gray', vmin=0, vmax=255)
    np.save(npy, grid)
    print(f'[map_builder] Mapa guardado: {png}')


def main():
    rclpy.init()
    node = MapBuilder()

    # Hilo de spin ROS en paralelo
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()

    # ── Figura ──────────────────────────────────────────────────────────────
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.canvas.manager.set_window_title('CapyTown RC3 — Mapa en tiempo real')

    # Imagen del mapa
    img_plot = ax.imshow(
        node.grid, cmap='gray', vmin=0, vmax=255,
        origin='upper',
        extent=[0, MAP_W, MAP_H, 0],
        interpolation='nearest'
    )

    # Borde de referencia de la pista
    pista_rect = mpatches.Rectangle(
        (0, 0), MAP_W, MAP_H,
        linewidth=2, edgecolor='cyan', facecolor='none', linestyle='--'
    )
    ax.add_patch(pista_rect)

    # Robot — punto + flecha de direccion
    robot_dot,  = ax.plot([], [], 'ro', markersize=7, zorder=5)
    arrow_arts  = []

    # Trayectoria
    traj_x, traj_y = [], []
    traj_line, = ax.plot([], [], 'b-', linewidth=1, alpha=0.5)

    # Líneas laterales (centrado)
    lat_izq_line, = ax.plot([], [], color='#AA00FF', lw=2.0, alpha=0.85, zorder=6)
    lat_der_line, = ax.plot([], [], color='#FF8800', lw=2.0, alpha=0.85, zorder=6)
    lat_text_arts = []   # textos y barra de centrado que se recrean cada tick

    ax.set_xlim(0, MAP_W)
    ax.set_ylim(MAP_H, 0)          # y invertido: 0 arriba, 1.80 abajo
    ax.set_xlabel('X (m) — Este')
    ax.set_ylabel('Y (m) — Sur')
    ax.set_title('Mapa de ocupacion CapyTown | gris=desconocido | blanco=libre | negro=pared')
    ax.grid(True, alpha=0.2, linestyle=':')

    # Leyenda de colores
    leyenda = [
        mpatches.Patch(color='white',   label='Libre'),
        mpatches.Patch(color='black',   label='Ocupado'),
        mpatches.Patch(color='gray',    label='Desconocido'),
        plt.Line2D([0],[0], color='#AA00FF', lw=2, label='Dist pared IZQ'),
        plt.Line2D([0],[0], color='#FF8800', lw=2, label='Dist pared DER'),
        plt.Line2D([0],[0], color='blue',    lw=1, label='Trayectoria'),
    ]
    ax.legend(handles=leyenda, loc='lower right', fontsize=8)

    def on_close(event):
        grid, _, __ = node.snapshot()
        guardar_mapa(grid)

    fig.canvas.mpl_connect('close_event', on_close)

    # Anchura del corredor (60 cm) — centro ideal = 30 cm de cada pared
    CORREDOR_W = 0.60
    CENTRO_IDEAL = CORREDOR_W / 2.0   # 0.30 m

    # ── Loop de visualizacion ────────────────────────────────────────────────
    try:
        while rclpy.ok():
            grid, (rx, ry, ryaw), (d_izq, d_der) = node.snapshot()
            img_plot.set_data(grid)

            # Posicion en metros del mapa (no en odom)
            mx = rx + ORIG_X
            my = -ry + ORIG_Y    # invertir eje y

            robot_dot.set_data([mx], [my])

            # Trayectoria
            traj_x.append(mx)
            traj_y.append(my)
            traj_line.set_data(traj_x, traj_y)

            # Flecha de direccion del robot
            for a in arrow_arts:
                a.remove()
            arrow_arts.clear()
            arrow_len = 0.08
            arr = ax.annotate(
                '', xy=(mx + arrow_len * math.cos(ryaw),
                        my - arrow_len * math.sin(ryaw)),
                xytext=(mx, my),
                arrowprops=dict(arrowstyle='->', color='red', lw=2)
            )
            arrow_arts.append(arr)

            # ── Líneas de centrado lateral ───────────────────────────────
            for art in lat_text_arts:
                try: art.remove()
                except Exception: pass
            lat_text_arts.clear()

            ang_izq = ryaw + math.pi * 0.5   # 90° izquierda
            ang_der = ryaw - math.pi * 0.5   # 90° derecha

            if math.isfinite(d_izq) and d_izq < 1.5:
                wl_x = mx + d_izq * math.cos(ang_izq)
                wl_y = my - d_izq * math.sin(ang_izq)   # y invertido en pantalla
                lat_izq_line.set_data([mx, wl_x], [my, wl_y])
                mid_x = (mx + wl_x) * 0.5
                mid_y = (my + wl_y) * 0.5
                t = ax.text(mid_x - 0.06, mid_y, f'IZQ\n{d_izq:.2f}m',
                            fontsize=7, color='#AA00FF', ha='center',
                            bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.8),
                            zorder=8)
                lat_text_arts.append(t)
            else:
                lat_izq_line.set_data([], [])

            if math.isfinite(d_der) and d_der < 1.5:
                wr_x = mx + d_der * math.cos(ang_der)
                wr_y = my - d_der * math.sin(ang_der)
                lat_der_line.set_data([mx, wr_x], [my, wr_y])
                mid_x = (mx + wr_x) * 0.5
                mid_y = (my + wr_y) * 0.5
                t = ax.text(mid_x + 0.06, mid_y, f'DER\n{d_der:.2f}m',
                            fontsize=7, color='#FF8800', ha='center',
                            bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.8),
                            zorder=8)
                lat_text_arts.append(t)
            else:
                lat_der_line.set_data([], [])

            # Barra de centrado + HUD
            if math.isfinite(d_izq) and math.isfinite(d_der):
                error = d_izq - CENTRO_IDEAL   # + = pegado a isla, - = pegado a pared ext
                color_err = '#AA00FF' if error > 0.03 else '#FF8800' if error < -0.03 else 'green'
                hud = ax.text(
                    0.02, 0.04,
                    f'IZQ {d_izq:.2f}m │ DER {d_der:.2f}m │ error {error*100:+.0f}cm',
                    transform=ax.transAxes, fontsize=9,
                    color=color_err, fontweight='bold',
                    bbox=dict(boxstyle='round', fc='white', alpha=0.85),
                    zorder=15)
                lat_text_arts.append(hud)

            fig.canvas.draw_idle()
            plt.pause(0.3)

    except KeyboardInterrupt:
        pass
    finally:
        grid, _, __ = node.snapshot()
        guardar_mapa(grid)
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.close('all')


if __name__ == '__main__':
    main()
