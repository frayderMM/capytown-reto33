#!/usr/bin/env python3
"""
viz_scan.py  —  Visualizador en vivo de /scan.

Uso:
    python3 viz_scan.py [--remove-min DEG] [--remove-max DEG] [--rango-max M]

Teclas: q cierra, r reinicia zoom
"""
import argparse
import math

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# sectores que usa behavior_fsm (para visualizarlos)
FRENTE_ANG = math.radians(40)
SIDE_LO    = math.radians(60)
SIDE_HI    = math.radians(120)


class ScanViz(Node):
    def __init__(self, rango_max):
        super().__init__("viz_scan")
        self.rango_max = rango_max
        self._msg = None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, "/scan", self._cb, qos)
        self.get_logger().info("viz_scan: esperando /scan ...")

    def _cb(self, msg):
        self._msg = msg


def procesar(msg, rango_max):
    validos, frontal, izq, der = [], [], [], []
    a0, da = msg.angle_min, msg.angle_increment
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r == 0.0:
            continue
        if r < msg.range_min or r > min(msg.range_max, rango_max):
            continue
        theta = a0 + i * da
        ath   = abs(theta)
        x, y  = r * math.cos(theta), r * math.sin(theta)
        validos.append((x, y))
        if abs(ath - math.pi) <= FRENTE_ANG:
            frontal.append((x, y))
        elif SIDE_LO <= ath <= SIDE_HI:
            (izq if theta > 0 else der).append((x, y))
    return validos, frontal, izq, der


def _sector(ax, theta_c, half, radio, color, alpha=0.12):
    ts = np.linspace(theta_c - half, theta_c + half, 40)
    xs = [0] + [radio * math.cos(t) for t in ts] + [0]
    ys = [0] + [radio * math.sin(t) for t in ts] + [0]
    ax.fill(xs, ys, color=color, alpha=alpha, zorder=1)


def dibujar(axes, validos, frontal, izq, der, rango_max):
    ax0, ax1 = axes
    M = rango_max

    for ax in axes:
        ax.cla()
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="white")
        for sp in ax.spines.values():
            sp.set_color("#333")
        ax.set_aspect("equal")
        ax.grid(True, color="#2a2a3e", linewidth=0.5)
        ax.set_xlim(-M, M)
        ax.set_ylim(-M, M)
        ax.set_xlabel("x [m]", color="#aaa")
        ax.set_ylabel("y [m]", color="#aaa")

    # ── panel izquierdo: todos los puntos coloreados por sector ──────────
    ax0.set_title(f"Scan — {len(validos)} pts válidos", color="white", fontsize=10)

    def scatter(ax, pts, color, zorder=3):
        if pts:
            ax.scatter([p[0] for p in pts], [p[1] for p in pts],
                       s=8, color=color, alpha=0.7, zorder=zorder)

    otros = [p for p in validos if p not in frontal and p not in izq and p not in der]
    scatter(ax0, otros,   "#546e7a")
    scatter(ax0, izq,     "#42a5f5")
    scatter(ax0, der,     "#66bb6a")
    scatter(ax0, frontal, "#ef5350")

    # ── panel derecho: puntos + sectores ─────────────────────────────────
    ax1.set_title("Sectores: rojo=FRENTE  azul=IZQ  verde=DER", color="white", fontsize=10)

    half_side = (SIDE_HI - SIDE_LO) / 2
    mid_side  = (SIDE_HI + SIDE_LO) / 2
    _sector(ax1,  math.pi,    FRENTE_ANG, M, "#ef5350")
    _sector(ax1, -math.pi,    FRENTE_ANG, M, "#ef5350")
    _sector(ax1,  mid_side,   half_side,  M, "#42a5f5")
    _sector(ax1, -mid_side,   half_side,  M, "#66bb6a")

    scatter(ax1, validos, "#66bb6a", zorder=4)

    # ── robot + flecha de frente (en ambos paneles) ───────────────────────
    for ax in axes:
        ax.plot(0, 0, "w^", markersize=11, zorder=8)
        ax.annotate("", xy=(-min(0.7, M * 0.25), 0), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color="#ef5350", lw=2.5),
                    zorder=9)
        ax.text(-min(0.85, M * 0.3), 0.07, "FRENTE",
                color="#ef5350", fontsize=8, fontweight="bold")

    # distancias mínimas por sector
    d_f = min((math.hypot(*p) for p in frontal), default=float('inf'))
    d_i = min((math.hypot(*p) for p in izq),     default=float('inf'))
    d_d = min((math.hypot(*p) for p in der),      default=float('inf'))

    df_s = f"{d_f:.2f}m" if math.isfinite(d_f) else "libre"
    di_s = f"{d_i:.2f}m" if math.isfinite(d_i) else "libre"
    dd_s = f"{d_d:.2f}m" if math.isfinite(d_d) else "libre"

    ax1.set_xlabel(
        f"FRENTE={df_s}  IZQ={di_s}  DER={dd_s}",
        color="white", fontsize=9
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--rango-max", type=float, default=3.5)
    args = p.parse_args()

    rclpy.init()
    nodo = ScanViz(args.rango_max)

    fig, axes = plt.subplots(1, 2, figsize=(13, 7))
    fig.patch.set_facecolor("#1a1a2e")
    plt.ion()
    plt.tight_layout()
    plt.show()

    try:
        while plt.fignum_exists(fig.number):
            # spin sin hilo separado — evita conflicto con Tk
            rclpy.spin_once(nodo, timeout_sec=0.05)
            if nodo._msg is not None:
                v, f, i, d = procesar(nodo._msg, args.rango_max)
                dibujar(axes, v, f, i, d, args.rango_max)
                fig.canvas.draw_idle()
            plt.pause(0.08)
    except KeyboardInterrupt:
        pass

    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
