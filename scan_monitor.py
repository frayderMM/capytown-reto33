#!/usr/bin/env python3
"""
scan_monitor.py — Monitor de LiDAR en terminal (sin pantalla/X11)

Muestra en tiempo real:
  - Distancia al FRENTE, IZQUIERDA, DERECHA
  - Barra visual ASCII de distancias
  - Alerta si algo está muy cerca

Uso:
    python3 scan_monitor.py

LiDAR: 0° = atrás, ±180° = frente
  theta positivo (~+90°) = izquierda
  theta negativo (~-90°) = derecha
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


FRENTE_ANGULO  = math.radians(40)   # ±40° alrededor de ±180°
LATERAL_LO     = math.radians(60)   # inicio sector lateral
LATERAL_HI     = math.radians(120)  # fin sector lateral
DIST_PELIGRO   = 0.30               # m — alerta roja
DIST_PRECAUCION= 0.50               # m — alerta amarilla
MAX_BARRA      = 2.0                # m — distancia máxima de la barra


def barra(dist, maximo=MAX_BARRA, ancho=30):
    if not math.isfinite(dist):
        return "░" * ancho + "  libre"
    proporcion = min(dist / maximo, 1.0)
    lleno = int(proporcion * ancho)
    return "█" * lleno + "░" * (ancho - lleno) + f"  {dist:.2f}m"


def color(dist):
    """Código ANSI de color según distancia."""
    if not math.isfinite(dist):
        return "\033[32m"   # verde
    if dist < DIST_PELIGRO:
        return "\033[31m"   # rojo
    if dist < DIST_PRECAUCION:
        return "\033[33m"   # amarillo
    return "\033[32m"       # verde


RESET = "\033[0m"
BOLD  = "\033[1m"
CLEAR = "\033[2J\033[H"


class ScanMonitor(Node):
    def __init__(self):
        super().__init__('scan_monitor')
        self.d_front  = float('inf')
        self.d_left   = float('inf')
        self.d_right  = float('inf')
        self.n_puntos = 0
        self.n_total  = 0
        self._ts      = time.time()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._cb, qos)
        self.create_timer(0.2, self._imprimir)
        self.get_logger().info('scan_monitor: esperando /scan ...')

    def _cb(self, msg):
        a0   = msg.angle_min
        da   = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max

        front = float('inf')
        d_pos = float('inf')
        d_neg = float('inf')
        validos = 0

        for i, r in enumerate(msg.ranges):
            if r == 0.0 or not math.isfinite(r) or r < rmin or r > rmax:
                continue
            validos += 1
            theta = a0 + i * da
            ath   = abs(theta)

            if abs(ath - math.pi) <= FRENTE_ANGULO:
                front = min(front, r)
            elif LATERAL_LO <= ath <= LATERAL_HI:
                if theta > 0:
                    d_pos = min(d_pos, r)
                else:
                    d_neg = min(d_neg, r)

        self.d_front  = front
        self.d_left   = d_pos
        self.d_right  = d_neg
        self.n_puntos = validos
        self.n_total  = len(msg.ranges)
        self._ts      = time.time()

    def _imprimir(self):
        edad = time.time() - self._ts
        sin_datos = edad > 1.0

        # encabezado
        sys.stdout.write(CLEAR)
        sys.stdout.write(f"{BOLD}═══ Monitor LiDAR — RC3 ═══{RESET}\n")
        if sin_datos:
            sys.stdout.write("\033[31mEsperando datos de /scan...\033[0m\n")
            sys.stdout.flush()
            return

        sys.stdout.write(
            f"Puntos válidos: {self.n_puntos}/{self.n_total}   "
            f"Actualizado hace: {edad*1000:.0f}ms\n"
        )
        sys.stdout.write("─" * 50 + "\n\n")

        # distancias
        cf = color(self.d_front)
        cl = color(self.d_left)
        cr = color(self.d_right)

        sys.stdout.write(f"  {BOLD}FRENTE{RESET}   {cf}{barra(self.d_front)}{RESET}\n")
        sys.stdout.write(f"  {BOLD}IZQUIERDA{RESET} {cl}{barra(self.d_left)}{RESET}\n")
        sys.stdout.write(f"  {BOLD}DERECHA{RESET}  {cr}{barra(self.d_right)}{RESET}\n")
        sys.stdout.write("\n")

        # mapa ASCII simple
        sys.stdout.write(_mapa_ascii(self.d_front, self.d_left, self.d_right))
        sys.stdout.write("\n")

        # estado
        frente_ok = self.d_front > 0.35
        izq_ok    = self.d_left  > 0.20
        der_ok    = self.d_right > 0.20

        if frente_ok:
            sys.stdout.write("\033[32m▶ FRENTE LIBRE — avanzaría\033[0m\n")
        elif izq_ok or der_ok:
            lado = "IZQUIERDA" if self.d_left >= self.d_right else "DERECHA"
            sys.stdout.write(f"\033[33m◀ FRENTE BLOQUEADO — giraría {lado}\033[0m\n")
        else:
            sys.stdout.write("\033[31m⚠ TODOS LOS LADOS BLOQUEADOS — 180°\033[0m\n")

        sys.stdout.flush()


def _mapa_ascii(front, left, right):
    """Mapa simplificado 5x5 con distancias."""
    def simbolo(d, umbral=0.35):
        if not math.isfinite(d): return "·"
        if d < umbral: return "█"
        if d < 0.60:   return "▒"
        return "·"

    sf = simbolo(front)
    sl = simbolo(left)
    sr = simbolo(right)

    lineas = [
        f"        [{sf}] Frente\n",
        f"         |\n",
        f"  [{sl}]--[R]--[{sr}]\n",
        f"  Izq          Der\n",
    ]
    return "".join(lineas)


def main():
    rclpy.init()
    nodo = ScanMonitor()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
