#!/usr/bin/env python3
"""
metrics_logger.py
-----------------
Nodo de METRICAS para el reto del Censo.

Compara las cajas detectadas (/cajas_avistadas) contra un ground-truth
declarado por parametro y calcula:
    VP  (verdaderos positivos)  : caja real detectada
    FP  (falsos positivos)      : deteccion sin caja real cerca
    FN  (falsos negativos)      : caja real no detectada
    tasa_deteccion = VP / (VP + FN)
    error_pos_prom_cm : distancia media VP <-> caja real (cm)
    dist_min_parada_cm: distancia minima al frenar frente a una caja (cm)

Al apagar el nodo (Ctrl+C) agrega una fila al archivo 'metricas_lidar.csv'.
colisiones y rodeo_exitoso los publica behavior_fsm.py en vivo
(/metrics/colisiones, /metrics/rodeo_exitoso) — este nodo solo guarda el
último valor recibido de cada uno al cerrar.

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, Int32, String


CAMPOS = [
    'corrida', 'fecha', 'cajas_reales', 'cajas_detectadas',
    'VP', 'FP', 'FN', 'tasa_deteccion',
    'error_pos_prom_cm', 'dist_min_parada_cm',
    'colisiones', 'rodeo_exitoso',
]


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('ground_truth', [1.0, 0.5, 2.0, -0.5, 3.0, 0.4])
        self.declare_parameter('tolerancia_match', 0.30)
        self.declare_parameter('corrida', 0)          # 0 = auto (cuenta filas en CSV)
        self.declare_parameter('archivo_csv', '~/metricas_lidar.csv')

        gt_plano = list(self.get_parameter('ground_truth').value)
        if len(gt_plano) % 2 != 0:
            self.get_logger().warn(
                f'ground_truth tiene {len(gt_plano)} valores (impar) — descartando el ultimo')
            gt_plano = gt_plano[:-1]
        self.gt  = [(gt_plano[i], gt_plano[i + 1])
                    for i in range(0, len(gt_plano), 2)]
        self.tol         = self.get_parameter('tolerancia_match').value
        self.archivo     = os.path.expanduser(
                               self.get_parameter('archivo_csv').value)
        corrida_param    = self.get_parameter('corrida').value
        self.corrida_num = corrida_param if corrida_param > 0 else self._siguiente_corrida()

        self.ultimas_detecciones = []
        self.dist_min_parada     = float('inf')   # se actualiza con /parada_dist
        self.colisiones          = 0     # último valor de /metrics/colisiones
        self.rodeo_exitoso       = '0/0'  # último valor de /metrics/rodeo_exitoso

        self.create_subscription(PoseArray, '/cajas_avistadas', self._cb_cajas,  10)
        self.create_subscription(Float32,   '/parada_dist',     self._cb_parada, 10)
        self.create_subscription(Int32, '/metrics/colisiones',    self._cb_colisiones, 10)
        self.create_subscription(String, '/metrics/rodeo_exitoso', self._cb_rodeo,      10)

        self.get_logger().info(
            f'metrics_logger iniciado | corrida={self.corrida_num} '
            f'ground-truth={len(self.gt)} cajas | CSV → {self.archivo}')

    # ── Helpers ───────────────────────────────────────────────────────────
    def _siguiente_corrida(self) -> int:
        """Cuenta las filas de datos existentes en el CSV para auto-numerar."""
        if not os.path.exists(self.archivo):
            return 1
        with open(self.archivo, newline='') as f:
            filas = sum(1 for r in csv.reader(f) if r)
        return max(1, filas)   # filas incluye header, así que corrida = filas

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _cb_cajas(self, msg: PoseArray):
        self.ultimas_detecciones = [
            (p.position.x, p.position.y) for p in msg.poses]

    def _cb_parada(self, msg: Float32):
        self.dist_min_parada = min(self.dist_min_parada, msg.data)

    def _cb_colisiones(self, msg: Int32):
        self.colisiones = msg.data

    def _cb_rodeo(self, msg: String):
        self.rodeo_exitoso = msg.data

    # ── Cálculo ───────────────────────────────────────────────────────────
    def _calcular(self):
        det = list(self.ultimas_detecciones)
        gt_libres = list(self.gt)
        vp, errores = 0, []

        for d in det:
            mejor_idx, mejor_dist = -1, self.tol
            for i, g in enumerate(gt_libres):
                dist = math.hypot(d[0] - g[0], d[1] - g[1])
                if dist < mejor_dist:
                    mejor_dist, mejor_idx = dist, i
            if mejor_idx >= 0:
                vp += 1
                errores.append(mejor_dist)
                gt_libres.pop(mejor_idx)

        fp   = len(det) - vp
        fn   = len(gt_libres)
        tasa = vp / (vp + fn) if (vp + fn) > 0 else 0.0
        err_cm = (sum(errores) / len(errores) * 100) if errores else 0.0
        return vp, fp, fn, tasa, err_cm

    # ── Guardar ───────────────────────────────────────────────────────────
    def guardar_csv(self):
        vp, fp, fn, tasa, err_cm = self._calcular()
        parada_cm = (self.dist_min_parada * 100
                     if math.isfinite(self.dist_min_parada) else -1)

        existe = os.path.exists(self.archivo)
        with open(self.archivo, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=CAMPOS)
            if not existe:
                w.writeheader()
            w.writerow({
                'corrida':            self.corrida_num,
                'fecha':              datetime.now().strftime('%Y-%m-%d %H:%M'),
                'cajas_reales':       len(self.gt),
                'cajas_detectadas':   len(self.ultimas_detecciones),
                'VP':                 vp,
                'FP':                 fp,
                'FN':                 fn,
                'tasa_deteccion':     round(tasa, 3),
                'error_pos_prom_cm':  round(err_cm, 1),
                'dist_min_parada_cm': round(parada_cm, 1),
                'colisiones':         self.colisiones,
                'rodeo_exitoso':      self.rodeo_exitoso,
            })

        self.get_logger().info(
            f'[corrida {self.corrida_num}] CSV → {self.archivo} | '
            f'VP={vp} FP={fp} FN={fn} tasa={tasa:.2f} '
            f'err={err_cm:.1f}cm parada={parada_cm:.1f}cm '
            f'colisiones={self.colisiones} rodeo_exitoso={self.rodeo_exitoso}')


def main(args=None):
    rclpy.init(args=args)
    nodo = MetricsLogger()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.guardar_csv()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
