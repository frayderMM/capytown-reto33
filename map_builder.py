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
        self._lock     = threading.Lock()

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

        for i in range(0, len(msg.ranges), RAY_STEP):
            r = msg.ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue

            raw_ang   = msg.angle_min + i * msg.angle_increment
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

    def snapshot(self):
        with self._lock:
            return self.grid.copy(), (self.robot_x, self.robot_y, self.robot_yaw)


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
    traj_line, = ax.plot([], [], 'b-', linewidth=1, alpha=0.5, label='Trayectoria')

    ax.set_xlim(0, MAP_W)
    ax.set_ylim(MAP_H, 0)          # y invertido: 0 arriba, 1.80 abajo
    ax.set_xlabel('X (m) — Este')
    ax.set_ylabel('Y (m) — Sur')
    ax.set_title('Mapa de ocupacion CapyTown | gris=desconocido | blanco=libre | negro=pared')
    ax.grid(True, alpha=0.2, linestyle=':')
    ax.legend(loc='upper right', fontsize=8)

    # Leyenda de colores
    leyenda = [
        mpatches.Patch(color='white',  label='Libre'),
        mpatches.Patch(color='black',  label='Ocupado'),
        mpatches.Patch(color='gray',   label='Desconocido'),
    ]
    ax.legend(handles=leyenda, loc='lower right', fontsize=8)

    def on_close(event):
        grid, _ = node.snapshot()
        guardar_mapa(grid)

    fig.canvas.mpl_connect('close_event', on_close)

    # ── Loop de visualizacion ────────────────────────────────────────────────
    try:
        while rclpy.ok():
            grid, (rx, ry, ryaw) = node.snapshot()
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
            # yaw: en odom, positivo = norte. En pantalla, norte = arriba = menor y.
            arr = ax.annotate(
                '', xy=(mx + arrow_len * math.cos(ryaw),
                        my - arrow_len * math.sin(ryaw)),   # - porque y invertido
                xytext=(mx, my),
                arrowprops=dict(arrowstyle='->', color='red', lw=2)
            )
            arrow_arts.append(arr)

            fig.canvas.draw_idle()
            plt.pause(0.3)

    except KeyboardInterrupt:
        pass
    finally:
        grid, _ = node.snapshot()
        guardar_mapa(grid)
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.close('all')


if __name__ == '__main__':
    main()
