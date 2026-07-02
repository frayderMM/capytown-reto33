#!/usr/bin/env python3
"""
lidar_viz.py — Monitor visual en tiempo real del reto (versión corregida).

Panel IZQUIERDO (marco del robot, frente hacia +x):
    · nube de puntos del /scan (ya rotada con --front, igual que el guardián)
    · segmentos detectados por el guardián coloreados por clase:
        CAJA = naranja · PARED = azul · ESQUINA = magenta · RUIDO = gris
    · footprint REAL del robot (15 cm frente / 10 cm atrás / 8 cm lados)
      y corredor frontal de colisión
    · cruz de distancias: 4 líneas perpendiculares a cada lado del robot
      (frente/atrás/izq/der) desde el BORDE físico hasta la pared más
      cercana en esa dirección, con la distancia en cm
    · estado de la FSM, distancia frontal y distancia a la pared derecha

Panel DERECHO (marco odom):
    · recorrido del robot + pose actual
    · cajas censadas por box_detector, numeradas
    · mapa FIJO de la pista: segmentos de pared ya detectados alguna vez,
      acumulados en marco odom (no desaparecen al girar/alejarse)

Consume /scan y /guardian/debug (JSON del guardián). Correr aparte (VNC):

    python3 lidar_viz.py                # frente en 180° (Yahboom MS200)
    python3 lidar_viz.py --front 0      # montaje estándar ROS
"""

import argparse
import json
import math
import threading

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

OFF_FRENTE, OFF_ATRAS, OFF_LADO = 0.15, 0.10, 0.08
BG, PANEL = '#1a1a2e', '#0f0e17'
COLORES = {'PARED': '#42a5f5', 'ESQUINA': '#ab47bc',
           'CAJA': '#ffa726', 'RUIDO': '#78909c'}
COLOR_RECORRIDO_REAL = '#26c6da'
COLOR_POSE = '#ffffff'


class VizNode(Node):
    def __init__(self):
        super().__init__('lidar_viz')
        self._lock = threading.Lock()
        self.scan = None
        self.debug = None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos)
        self.create_subscription(String, '/guardian/debug', self._cb_dbg, 10)
        self.get_logger().info('lidar_viz: suscrito a /scan y /guardian/debug')

    def _cb_scan(self, msg):
        with self._lock:
            self.scan = msg

    def _cb_dbg(self, msg):
        with self._lock:
            try:
                self.debug = json.loads(msg.data)
            except json.JSONDecodeError:
                pass

    def snapshot(self):
        with self._lock:
            return self.scan, self.debug


def puntos_scan(msg, front_rad, rmax, atras_rad):
    pts = []
    lim = math.pi - atras_rad
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < msg.range_min or r > min(msg.range_max, rmax):
            continue
        af = math.atan2(math.sin(msg.angle_min + i * msg.angle_increment - front_rad),
                        math.cos(msg.angle_min + i * msg.angle_increment - front_rad))
        if abs(af) > lim:
            continue
        pts.append((r * math.cos(af), r * math.sin(af)))
    return pts


def estilo(ax, titulo):
    ax.set_facecolor(PANEL)
    ax.tick_params(colors='white')
    ax.spines[:].set_color('#333')
    ax.set_aspect('equal')
    ax.grid(True, color='#1e1e2e', linewidth=0.6)
    ax.set_xlabel('x [m]', color='#aaa')
    ax.set_ylabel('y [m]', color='#aaa')
    ax.set_title(titulo, color='white', fontsize=10)


def dibujar_robot(ax):
    ax.add_patch(mpatches.Rectangle(
        (-OFF_ATRAS, -OFF_LADO), OFF_ATRAS + OFF_FRENTE, 2 * OFF_LADO,
        fill=False, edgecolor='white', linewidth=1.4, zorder=6))
    ax.plot([OFF_FRENTE - 0.03, OFF_FRENTE, OFF_FRENTE - 0.03],
            [-0.04, 0.0, 0.04], color='white', linewidth=1.4, zorder=6)
    ax.plot(0, 0, 'w.', markersize=4, zorder=7)
    semi = OFF_LADO + 0.06
    ax.add_patch(mpatches.Rectangle(
        (OFF_FRENTE, -semi), 0.35, 2 * semi,
        facecolor='#ef5350', alpha=0.10, edgecolor='#ef5350',
        linewidth=0.7, linestyle='--', zorder=1))


def dibujar_cruz_distancias(ax, dbg):
    """Cuatro líneas perpendiculares a los lados del robot (forma de +),
    desde el BORDE físico (no el LiDAR) hasta la pared/obstáculo más
    cercano en esa dirección — frente, atrás, izquierda y derecha —
    con la distancia real en cm rotulada en la punta."""
    LARGO_MAX = 1.0   # recorta la línea dibujada para no salirse del plot;
                       # la etiqueta siempre muestra la distancia real
    specs = [
        ('d_frente', (OFF_FRENTE, 0.0), (1, 0)),
        ('d_atras',  (-OFF_ATRAS, 0.0), (-1, 0)),
        ('d_izq',    (0.0, OFF_LADO), (0, 1)),
        ('d_der',    (0.0, -OFF_LADO), (0, -1)),
    ]
    for clave, (bx, by), (dx, dy) in specs:
        d = dbg.get(clave)
        if d is None or d <= 0:
            continue
        d_dibujo = min(d, LARGO_MAX)
        ex, ey = bx + dx * d_dibujo, by + dy * d_dibujo
        ax.plot([bx, ex], [by, ey], color='#ffd54f', linewidth=1.3,
                linestyle=':', zorder=5)
        ax.plot(ex, ey, 'o', color='#ffd54f', markersize=4, zorder=6)
        ax.annotate(f'{d * 100:.0f}cm', (ex, ey), color='#ffd54f',
                    fontsize=7.5, fontweight='bold',
                    ha='left' if dx >= 0 else 'right',
                    va='bottom' if dy >= 0 else 'top', zorder=6,
                    bbox=dict(boxstyle='round,pad=0.12', fc=PANEL,
                              ec='#ffd54f', linewidth=0.5, alpha=0.85))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--front', type=float, default=180.0,
                   help='ángulo crudo del frente del LiDAR (Yahboom=180)')
    p.add_argument('--excluir-atras', type=float, default=60.0)
    p.add_argument('--rango-max', type=float, default=3.5)
    args, _ = p.parse_known_args()
    front_rad = math.radians(args.front)
    atras_rad = math.radians(args.excluir_atras) / 2.0

    rclpy.init()
    nodo = VizNode()
    threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True).start()

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 6.5))
    fig.patch.set_facecolor(BG)
    plt.ion()
    plt.show()

    try:
        while plt.fignum_exists(fig.number):
            scan, dbg = nodo.snapshot()

            axL.cla()
            estilo(axL, 'Marco robot — scan + clasificación')
            axR.cla()
            estilo(axR, 'Marco odom — recorrido + cajas')

            if scan is not None:
                pts = puntos_scan(scan, front_rad, args.rango_max, atras_rad)
                if pts:
                    axL.scatter([q[0] for q in pts], [q[1] for q in pts],
                                s=5, color='#546e7a', alpha=0.5, zorder=2)
                    M = min(max(max(abs(x), abs(y)) for x, y in pts) + 0.3, 1.2)
                    axL.set_xlim(-M, M)
                    axL.set_ylim(-M, M)
            dibujar_robot(axL)

            if dbg is not None:
                dibujar_cruz_distancias(axL, dbg)
                vistos = set()
                for s in dbg.get('segs', []):
                    if s['clase'] == 'RUIDO':
                        continue
                    c = COLORES.get(s['clase'], '#888')
                    lbl = s['clase'] if s['clase'] not in vistos else None
                    vistos.add(s['clase'])
                    ancho = 4.0 if s['clase'] == 'CAJA' else 2.6
                    axL.plot([s['x1'], s['x2']], [s['y1'], s['y2']],
                             color=c, linewidth=ancho, zorder=4, label=lbl)
                    mx = 0.5 * (s['x1'] + s['x2'])
                    my = 0.5 * (s['y1'] + s['y2'])
                    axL.annotate(f"{s['lon'] * 100:.0f}cm", (mx, my),
                                 color=c, fontsize=6.5, ha='center',
                                 va='bottom', zorder=6)
                if vistos:
                    axL.legend(loc='upper right', facecolor=BG,
                               edgecolor='#444', labelcolor='white', fontsize=8)

                df, pd_ = dbg.get('d_frente'), dbg.get('pared_der')
                frente_txt = '--' if df is None else f'{df:.2f} m'
                clase_txt = dbg.get('clase_frente') or '--'
                pared_txt = '--' if pd_ is None else f"{pd_['d']:.2f} m"
                ref_txt = '' if pd_ is None else f" [{pd_.get('tipo', 'PARED')}]"
                accion_txt = dbg.get('accion', '---')
                axL.text(0.02, 0.02,
                         f"FSM: {dbg['estado']}[{dbg['fase']}]   "
                         f"frente: {frente_txt} ({clase_txt})   "
                         f"pared der: {pared_txt}{ref_txt}",
                         transform=axL.transAxes, color='#ffd54f', fontsize=9)
                axL.text(0.02, 0.96, f"ACCION: {accion_txt}",
                         transform=axL.transAxes, color='#80ffea',
                         fontsize=11, fontweight='bold', va='top')

                mapa_pared = dbg.get('mapa_pared', [])
                if mapa_pared:
                    axR.scatter([p[0] for p in mapa_pared], [p[1] for p in mapa_pared],
                                s=3, color=COLORES['PARED'], alpha=0.6, zorder=2,
                                label='mapa pista')

                trail = dbg.get('trail', [])
                if trail:
                    axR.plot([q[0] for q in trail], [q[1] for q in trail],
                             color=COLOR_RECORRIDO_REAL, linewidth=2.2,
                             alpha=0.95, zorder=3, label='recorrido real')
                pose = dbg.get('pose')
                if pose:
                    x, y, yaw = pose
                    axR.plot(x, y, marker=(3, 0, math.degrees(yaw) - 90),
                             markersize=13, color=COLOR_POSE, zorder=5,
                             label='carrito')
                cajas_vivas = dbg.get('cajas_vivas', [])
                for i, (bx, by) in enumerate(cajas_vivas):
                    axR.add_patch(mpatches.Rectangle(
                        (bx - 0.10, by - 0.10), 0.20, 0.20,
                        facecolor='none', edgecolor='#ffa726',
                        linewidth=2.0, linestyle='--', zorder=4))
                    axR.annotate(f'VIVA {i + 1}', (bx, by), color='#ffa726',
                                 ha='center', va='center', fontsize=7,
                                 fontweight='bold', zorder=5)
                cajas_fijas = dbg.get('cajas_fijas', [])
                for i, (bx, by) in enumerate(cajas_fijas):
                    axR.add_patch(mpatches.Rectangle(
                        (bx - 0.10, by - 0.10), 0.20, 0.20,
                        facecolor='#ffa726', alpha=0.55, edgecolor='#ffa726',
                        linewidth=2.0, zorder=4,
                        label='caja censada (der)' if i == 0 else None))
                    axR.annotate(f'{i + 1}', (bx, by), color='#1a1a2e',
                                 ha='center', va='center', fontsize=8,
                                 fontweight='bold', zorder=5)
                if trail or pose or cajas_vivas or cajas_fijas or mapa_pared:
                    handles, labels = axR.get_legend_handles_labels()
                    if cajas_vivas:
                        handles.append(mpatches.Patch(
                            facecolor='none', edgecolor='#ffa726',
                            linestyle='--', label='caja visible'))
                    axR.legend(handles=handles, loc='upper right', facecolor=BG,
                               edgecolor='#444', labelcolor='white', fontsize=8)
                axR.set_title(
                    f"Recorrido + cajas vivas — {len(cajas_vivas)} visibles",
                              color='white', fontsize=10)
                axR.text(0.02, 0.02,
                         f'Cajas censadas (lado derecho): {len(cajas_fijas)}',
                         transform=axR.transAxes, color='#ffa726', fontsize=9,
                         fontweight='bold',
                         bbox=dict(boxstyle='round,pad=0.25', fc=PANEL,
                                   ec='#ffa726', linewidth=0.6, alpha=0.85))
                todos = trail + cajas_vivas + cajas_fijas + mapa_pared + \
                        ([pose[:2]] if pose else [])
                if todos:
                    xs, ys = [q[0] for q in todos], [q[1] for q in todos]
                    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
                    R = (max(max(xs) - min(xs), max(ys) - min(ys)) / 2 + 1.0) * 1.5
                    R = max(R, 3.3)  # la pista completa es ~3.0x1.8 m: no
                                     # arrancar "pegado" al robot, dejar
                                     # margen de sobra para verla entera
                    axR.set_xlim(cx - R, cx + R)
                    axR.set_ylim(cy - R, cy + R)
            else:
                axR.text(0.5, 0.5, 'esperando /guardian/debug…',
                         transform=axR.transAxes, color='#888',
                         ha='center', fontsize=10)

            fig.canvas.draw_idle()
            plt.pause(0.12)
    except KeyboardInterrupt:
        pass

    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
