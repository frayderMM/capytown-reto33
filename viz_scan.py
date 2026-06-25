#!/usr/bin/env python3
"""
viz_scan.py  —  Visualizador en vivo de /scan con eliminación de rango angular.

Uso:
    python3 viz_scan.py [--remove-min DEG] [--remove-max DEG] [--rango-max M]

Ejemplos:
    python3 viz_scan.py                          # sin filtro angular extra
    python3 viz_scan.py --remove-min 80 --remove-max 280   # quita zona trasera
    python3 viz_scan.py --remove-min -30 --remove-max 30   # quita zona frontal

Teclas matplotlib:
    r   reinicia el zoom
    q   cierra
"""

import argparse
import math
import threading

import matplotlib
matplotlib.use("TkAgg")           # o "Qt5Agg" si tu sistema no tiene Tk
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# ─── parámetros de estilo ────────────────────────────────────────────────────
COLOR_RAW      = "#4fc3f7"   # azul claro  → puntos antes del filtro angular
COLOR_FILT     = "#ef5350"   # rojo        → puntos eliminados por el filtro
COLOR_VALIDO   = "#66bb6a"   # verde       → puntos que pasan todos los filtros
ALPHA          = 0.55
PUNTO_TAM      = 8


# ─── nodo ROS 2 ──────────────────────────────────────────────────────────────
class ScanViz(Node):
    def __init__(self, remove_min_rad, remove_max_rad, rango_max):
        super().__init__("viz_scan")
        self.remove_min = remove_min_rad   # ángulo inicio de la zona eliminada (rad)
        self.remove_max = remove_max_rad   # ángulo fin   de la zona eliminada (rad)
        self.rango_max  = rango_max        # distancia máxima a mostrar (m)

        self._lock  = threading.Lock()
        self._datos = None                 # último mensaje /scan

        self.create_subscription(LaserScan, "/scan", self._cb_scan, 10)
        if remove_min_rad is not None and remove_max_rad is not None:
            zona = f"[{math.degrees(remove_min_rad):.1f}°, {math.degrees(remove_max_rad):.1f}°]"
        else:
            zona = "ninguna"
        self.get_logger().info(f"Suscrito a /scan — zona eliminada: {zona}")

    def _cb_scan(self, msg: LaserScan):
        with self._lock:
            self._datos = msg

    def get_datos(self):
        with self._lock:
            return self._datos


# ─── lógica de filtrado ───────────────────────────────────────────────────────
def procesar_scan(msg: LaserScan, remove_min, remove_max, rango_max):
    """Devuelve tres listas de (x, y) en frame base_link:
        validos  — pasan sensor + rango angular OK
        eliminados — eliminados solo por el filtro angular (pero sensor OK)
    """
    validos, eliminados = [], []

    n = len(msg.ranges)
    for i, r in enumerate(msg.ranges):
        # --- filtro sensor: inf / nan / rango físico ---
        if not math.isfinite(r):
            continue
        if r < msg.range_min or r > min(msg.range_max, rango_max):
            continue

        theta = msg.angle_min + i * msg.angle_increment

        # --- normaliza theta a [-pi, pi] ---
        theta_n = math.atan2(math.sin(theta), math.cos(theta))

        # --- filtro angular personalizado ---
        # comprueba si theta_n cae en el arco [remove_min, remove_max]
        en_zona = _en_arco(theta_n, remove_min, remove_max)

        x = r * math.cos(theta)
        y = r * math.sin(theta)

        if en_zona:
            eliminados.append((x, y))
        else:
            validos.append((x, y))

    return validos, eliminados


def _en_arco(theta, a_min, a_max):
    """True si theta cae dentro del arco [a_min, a_max] (en [-pi,pi])."""
    if a_min is None or a_max is None:
        return False
    if a_min <= a_max:
        return a_min <= theta <= a_max
    else:
        # arco cruza ±pi
        return theta >= a_min or theta <= a_max


# ─── matplotlib ──────────────────────────────────────────────────────────────
def construir_figura():
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    fig.patch.set_facecolor("#1a1a2e")
    for ax in axes:
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="white")
        ax.spines[:].set_color("#333")
        ax.set_aspect("equal")
        ax.grid(True, color="#2a2a3e", linewidth=0.5)

    axes[0].set_title("Scan completo (verde=válido, rojo=eliminado)",
                       color="white", fontsize=10)
    axes[1].set_title("Scan filtrado (solo puntos válidos)",
                       color="white", fontsize=10)
    for ax in axes:
        ax.set_xlabel("x [m]", color="#aaa")
        ax.set_ylabel("y [m]", color="#aaa")

    # marcador del robot
    for ax in axes:
        ax.plot(0, 0, "w^", markersize=10, zorder=5, label="robot")
        ax.annotate("robot", (0, 0), color="white", fontsize=8,
                    xytext=(0.05, 0.05), textcoords="data")

    fig.tight_layout()
    return fig, axes


def actualizar_figura(axes, validos, eliminados):
    for ax in axes:
        # conserva robot marker (último artista) y borra el resto
        artistas = ax.get_lines() + ax.collections
        for a in artistas:
            a.remove()

    # eje izquierdo — todos
    todos_x = [p[0] for p in validos + eliminados]
    todos_y = [p[1] for p in validos + eliminados]
    ax0 = axes[0]
    if eliminados:
        ex = [p[0] for p in eliminados]
        ey = [p[1] for p in eliminados]
        ax0.scatter(ex, ey, s=PUNTO_TAM, color=COLOR_FILT,
                    alpha=ALPHA, label="eliminados", zorder=3)
    if validos:
        vx = [p[0] for p in validos]
        vy = [p[1] for p in validos]
        ax0.scatter(vx, vy, s=PUNTO_TAM, color=COLOR_VALIDO,
                    alpha=ALPHA, label="válidos", zorder=4)
    ax0.plot(0, 0, "w^", markersize=10, zorder=5)

    # eje derecho — solo válidos
    ax1 = axes[1]
    if validos:
        ax1.scatter(vx, vy, s=PUNTO_TAM, color=COLOR_VALIDO,
                    alpha=ALPHA, zorder=4)
    ax1.plot(0, 0, "w^", markersize=10, zorder=5)

    # ajusta límites dinámicamente
    if todos_x:
        M = max(abs(v) for v in todos_x + todos_y) + 0.3
        for ax in axes:
            ax.set_xlim(-M, M)
            ax.set_ylim(-M, M)


# ─── main ────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Visualizador de /scan con filtro angular")
    p.add_argument("--remove-min", type=float, default=None,
                   help="Ángulo inicial de la zona a eliminar (grados, -180..180)")
    p.add_argument("--remove-max", type=float, default=None,
                   help="Ángulo final  de la zona a eliminar (grados, -180..180)")
    p.add_argument("--rango-max", type=float, default=5.0,
                   help="Distancia máxima a visualizar (m, default 5.0)")
    return p.parse_args()


def main():
    args = parse_args()

    rm_min = math.radians(args.remove_min) if args.remove_min is not None else None
    rm_max = math.radians(args.remove_max) if args.remove_max is not None else None

    rclpy.init()
    nodo = ScanViz(rm_min, rm_max, args.rango_max)

    # hilo ROS en background
    hilo = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo.start()

    fig, axes = construir_figura()
    plt.ion()
    plt.show()

    info_txt = fig.text(
        0.01, 0.01,
        f"filtro angular: [{args.remove_min}°, {args.remove_max}°]  |  rango máx: {args.rango_max} m",
        color="#aaa", fontsize=8
    )
    n_txt = fig.text(0.5, 0.01, "", color="white", fontsize=9, ha="center")

    try:
        while plt.fignum_exists(fig.number):
            msg = nodo.get_datos()
            if msg is not None:
                validos, eliminados = procesar_scan(msg, rm_min, rm_max, args.rango_max)
                actualizar_figura(axes, validos, eliminados)
                n_txt.set_text(
                    f"válidos: {len(validos)}  eliminados: {len(eliminados)}  "
                    f"total sensor-OK: {len(validos)+len(eliminados)}"
                )
                fig.canvas.draw_idle()
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass

    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
