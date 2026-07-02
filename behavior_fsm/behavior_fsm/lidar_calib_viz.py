#!/usr/bin/env python3
"""
lidar_calib_viz.py  —  Monitor de calibracion para el alineado con la pared derecha.

Lee /scan directamente (no depende de que wall_follower este corriendo) y
dibuja:
  - la nube de puntos del LiDAR (vista superior, frente = arriba)
  - los dos haces que usa wall_follower (frente-derecha y derecha pura)
  - alpha (angulo respecto a la pared), r_frente, r_derecha, distancia a
    la pared y el estado (PARALELO / GIRA IZQ / GIRA DER)

Sirve para la prueba ESTATICA: robot quieto junto a la pared, se gira a
mano y se observa en vivo si alpha se mueve como se espera (crece hacia un
lado, se acerca a 0 cuando el robot queda paralelo). Tambien ayuda a ver
cuando el LiDAR necesita calibracion: si con el robot visiblemente paralelo
a la pared alpha NO marca ~0, el offset angular del propio LiDAR (montaje)
esta desviado y hay que corregir 'angulo_derecha_deg'/'angulo_frente_deg'.

Uso:
    ros2 run behavior_fsm lidar_calib_viz
    ros2 run behavior_fsm lidar_calib_viz --ros-args --params-file config/params.yaml
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

BG = '#0d1117'
PANEL = '#161b22'
C_PTS = '#3a4a5a'
C_FRENTE = '#4f8ef7'
C_DERECHA = '#2ecc71'
C_OK = '#2ecc71'
C_WARN = '#e94560'
C_TEXT = '#e6edf3'


class LidarCalibViz(Node):

    def __init__(self):
        super().__init__('lidar_calib_viz')

        self.declare_parameter('angulo_frente_deg', -50.0)
        self.declare_parameter('angulo_derecha_deg', -90.0)
        self.declare_parameter('ventana_busqueda_deg', 3.0)
        self.declare_parameter('umbral_paralelo_deg', 3.0)
        self.declare_parameter('rango_max_plot', 3.0)  # m, alcance dibujado

        self._ang_frente = math.radians(self.get_parameter('angulo_frente_deg').value)
        self._ang_derecha = math.radians(self.get_parameter('angulo_derecha_deg').value)
        self._ventana = math.radians(self.get_parameter('ventana_busqueda_deg').value)
        self._umbral_deg = self.get_parameter('umbral_paralelo_deg').value
        self._rmax_plot = self.get_parameter('rango_max_plot').value
        self._theta = abs(self._ang_derecha - self._ang_frente)

        self.ultimo_scan = None
        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)

    def _cb_scan(self, msg: LaserScan):
        self.ultimo_scan = msg

    def _rango_en_angulo(self, msg: LaserScan, angulo_obj: float):
        idx_obj = int(round((angulo_obj - msg.angle_min) / msg.angle_increment))
        medio = max(1, int(round(self._ventana / msg.angle_increment)))
        mejor_r, mejor_di = None, None
        for di in range(-medio, medio + 1):
            i = idx_obj + di
            if i < 0 or i >= len(msg.ranges):
                continue
            r = msg.ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if mejor_di is None or abs(di) < mejor_di:
                mejor_r, mejor_di = r, abs(di)
        return mejor_r

    def calcular(self):
        """Devuelve (puntos_xy, r_frente, r_derecha, alpha_deg, dist_pared, estado) o None."""
        msg = self.ultimo_scan
        if msg is None:
            return None

        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > min(msg.range_max, self._rmax_plot):
                continue
            ang = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(ang), r * math.sin(ang)))

        r_frente = self._rango_en_angulo(msg, self._ang_frente)
        r_derecha = self._rango_en_angulo(msg, self._ang_derecha)
        if r_frente is None or r_derecha is None:
            return pts, None, None, None, None, 'SIN LECTURA DERECHA'

        alpha = math.atan2(r_frente * math.cos(self._theta) - r_derecha,
                           r_frente * math.sin(self._theta))
        alpha_deg = math.degrees(alpha)
        dist_pared = r_derecha * math.cos(alpha)
        estado = 'PARALELO' if abs(alpha_deg) <= self._umbral_deg else (
            'GIRA IZQUIERDA' if alpha > 0 else 'GIRA DERECHA')
        return pts, r_frente, r_derecha, alpha_deg, dist_pared, estado


def main(args=None):
    rclpy.init(args=args)
    nodo = LidarCalibViz()

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 7))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(PANEL)

    try:
        while rclpy.ok():
            rclpy.spin_once(nodo, timeout_sec=0.05)
            resultado = nodo.calcular()
            if resultado is None:
                plt.pause(0.01)
                continue

            pts, r_frente, r_derecha, alpha_deg, dist_pared, estado = resultado
            ax.clear()
            ax.set_facecolor(PANEL)
            rmax = nodo._rmax_plot

            if pts:
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                ax.scatter(ys, xs, s=3, c=C_PTS)  # y a la izquierda en x, x-robot hacia arriba

            ax.scatter([0], [0], s=60, c='#f5c518', marker='^', zorder=5)

            if r_frente is not None:
                xf = r_frente * math.cos(nodo._ang_frente)
                yf = r_frente * math.sin(nodo._ang_frente)
                ax.plot([0, yf], [0, xf], c=C_FRENTE, lw=2, label='haz frente-derecha')
            if r_derecha is not None:
                xd = r_derecha * math.cos(nodo._ang_derecha)
                yd = r_derecha * math.sin(nodo._ang_derecha)
                ax.plot([0, yd], [0, xd], c=C_DERECHA, lw=2, label='haz derecha')

            color_estado = C_OK if estado == 'PARALELO' else C_WARN
            titulo = estado
            if alpha_deg is not None:
                titulo += f'   alpha={alpha_deg:+.1f} deg'
            if dist_pared is not None:
                titulo += f'   dist={dist_pared:.2f} m'
            ax.set_title(titulo, color=color_estado, fontsize=13, fontweight='bold')

            ax.set_xlim(-rmax, rmax)
            ax.set_ylim(-0.5, rmax)
            ax.set_xlabel('y (izquierda +)', color=C_TEXT)
            ax.set_ylabel('x (frente +)', color=C_TEXT)
            ax.tick_params(colors=C_TEXT)
            ax.legend(loc='upper left', fontsize=8, facecolor=PANEL, labelcolor=C_TEXT)
            ax.set_aspect('equal')
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
