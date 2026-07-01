#!/usr/bin/env python3
"""
lidar_utils.py — Funciones PURAS de percepción para el censo (Parte A).

CORRECCIONES respecto a la versión anterior:

1. MARCO DEL ROBOT. El MS200 va montado con el frente en raw=180°: la
   versión anterior convertía a cartesiano SIN rotar (lidar_front_deg),
   así que los centroides quedaban en un marco girado 180° y, al
   componerlos con /odom_raw, el censo caía en posiciones espejadas.
   Ahora todo punto se rota primero a base_link (x adelante, y izquierda).

2. VALIDACIÓN POR LADOS (Split-and-Merge corregido, sin el bug del [1:]),
   no solo por "ancho extremo a extremo": una caja de 20×20 vista en L
   tiene dos lados de ≤20 cm; su ancho extremo-a-extremo puede llegar a
   28 cm (diagonal) y una pared fragmentada podía colarse con la regla
   antigua. Se acepta como caja el cluster cuyos lados visibles son TODOS
   cortos; cualquier lado largo (pared/esquina) lo descarta.
"""

import math
from typing import List, Optional, Tuple

Point = Tuple[float, float]


def normalizar(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def filtrar_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 front_rad: float, rango_max: float,
                 excluir_atras_rad: float = 0.0):
    """/scan crudo → lista de (idx, x, y) en base_link, orden de barrido
    CONTINUO alrededor del robot (ver nota de rotacion abajo)."""
    n = len(ranges)
    lim = math.pi - excluir_atras_rad
    # Igual que en behavior_fsm/percepcion.py: el array crudo "da la vuelta"
    # en raw=angle_min, y con front_rad=180° (cable atras del MS200) eso
    # cae en af=0 -- el FRENTE -- partiendo en dos clusters cualquier caja
    # que el robot tenga justo enfrente (pre_segmentar depende del indice
    # para detectar contigueidad). Rotar el barrido para arrancar "detras"
    # del robot mueve ese corte al cono trasero, ya excluido.
    i0 = int(round((front_rad + math.pi - angle_min) / angle_increment)) % n
    pts = []
    for k in range(n):
        i = (i0 + k) % n
        r = ranges[i]
        if not math.isfinite(r):
            continue
        if r < range_min or r > min(range_max, rango_max):
            continue
        af = normalizar(angle_min + i * angle_increment - front_rad)
        if abs(af) > lim:
            continue
        pts.append((k, r * math.cos(af), r * math.sin(af)))
    return pts


def pre_segmentar(pts_idx, salto_dist: float, salto_idx: int):
    """Clustering: corta por salto euclidiano o hueco de índices."""
    if not pts_idx:
        return []
    grupos, actual = [], [pts_idx[0]]
    for k in range(1, len(pts_idx)):
        ip, xp, yp = pts_idx[k - 1]
        ic, xc, yc = pts_idx[k]
        if (ic - ip) > salto_idx or math.hypot(xc - xp, yc - yp) > salto_dist:
            if len(actual) >= 2:
                grupos.append([(x, y) for _, x, y in actual])
            actual = [pts_idx[k]]
        else:
            actual.append(pts_idx[k])
    if len(actual) >= 2:
        grupos.append([(x, y) for _, x, y in actual])
    return grupos


def _dist_perp(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return math.hypot(px - ax, py - ay)
    return abs(dy * px - dx * py + bx * ay - by * ax) / L


def split(pts: List[Point], umbral: float):
    if len(pts) < 2:
        return [pts]
    ax, ay = pts[0]
    bx, by = pts[-1]
    dists = [_dist_perp(p[0], p[1], ax, ay, bx, by) for p in pts]
    im = max(range(len(dists)), key=lambda i: dists[i])
    if dists[im] > umbral and len(pts) > 2:
        # sin [1:] — el bug anterior descartaba un sub-grupo por división
        return split(pts[:im + 1], umbral) + split(pts[im:], umbral)
    return [pts]


def merge(grupos, umbral: float):
    if len(grupos) <= 1:
        return grupos
    out = [grupos[0]]
    for g in grupos[1:]:
        cand = out[-1] + g
        if len(split(cand, umbral)) == 1:
            out[-1] = cand
        else:
            out.append(g)
    return out


def lados_cluster(grupo: List[Point], umbral_split: float, min_pts: int):
    """Longitudes de los lados rectos del cluster (Split-and-Merge)."""
    lados = []
    for s in merge(split(grupo, umbral_split), umbral_split):
        if len(s) < min_pts:
            continue
        lon = math.hypot(s[-1][0] - s[0][0], s[-1][1] - s[0][1])
        if lon >= 0.05:
            lados.append(lon)
    return lados


def es_caja(grupo: List[Point], umbral_split: float, min_pts: int,
            lado_min: float, lado_max: float) -> bool:
    """Caja ⇔ tiene lados visibles y TODOS están en [lado_min, lado_max]."""
    lados = lados_cluster(grupo, umbral_split, min_pts)
    if not lados:
        return False
    return all(lado_min <= l <= lado_max for l in lados)


def centroide(grupo: List[Point]) -> Point:
    n = len(grupo)
    return (sum(p[0] for p in grupo) / n, sum(p[1] for p in grupo) / n)


def componer_odom(x_r, y_r, px, py, pyaw) -> Point:
    """base_link → odom:  R(yaw)·p + t."""
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (c * x_r - s * y_r + px, s * x_r + c * y_r + py)


def yaw_desde_quaternion(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def distancia(p1: Point, p2: Point) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
