#!/usr/bin/env python3
"""
percepcion.py — Percepción LiDAR compartida del reto CapyTown (funciones PURAS).

Este módulo es la ÚNICA fuente de verdad de la percepción: la usan
behavior_fsm.py (guardián), wall_follower.py (debug) y su lógica está
replicada en box_detector/lidar_utils.py (Parte A). Ser puras (sin ROS)
permite probarlas offline.

CORRECCIONES clave respecto a la versión anterior:

1. MARCO DEL ROBOT UNIFICADO. El MS200 del Yahboom va montado con el
   frente en raw=180° (cable atrás). Antes solo behavior_fsm rotaba los
   ángulos; wall_follower y box_detector procesaban el scan crudo, así
   que su "derecha/izquierda" y las posiciones del censo salían en un
   marco girado 180°. Aquí TODO punto se lleva primero a base_link:
       af = normalizar(raw − front_rad)   →   x = r·cos(af) (adelante)
                                              y = r·sin(af) (izquierda)
   Con front=180°: pared derecha (raw≈+90°) → af≈−90° → y<0. Correcto.

2. SPLIT-AND-MERGE SIN EL BUG DEL [1:]. La versión previa hacía
   `_split(izq) + _split(der)[1:]`, que descarta el PRIMER SUB-GRUPO
   completo del lado derecho de cada división — en una esquina, una de
   las dos paredes desaparecía. Por eso "medir el ancho de lo que
   bloquea el frente resultó poco confiable y confundía caja con
   esquina": la esquina se veía como un solo lado. Corregido, la
   clasificación caja/esquina vuelve a ser viable y defendible.

3. CLASIFICACIÓN GEOMÉTRICA POR LADOS (regla del reto):
       CAJA    → todos los lados visibles ≤ lado_caja_max (~20 cm + tol;
                 la diagonal máxima de una caja de 20×20 es 28 cm)
       ESQUINA → dos lados ≥ min_long_pared y ~perpendiculares (90°±25°)
       PARED   → al menos un lado ≥ min_long_pared
       RUIDO   → resto
"""

import math
from typing import List, Optional, Tuple

Point = Tuple[float, float]

CAJA, PARED, ESQUINA, RUIDO = 'CAJA', 'PARED', 'ESQUINA', 'RUIDO'


# ── Marco y filtrado ─────────────────────────────────────────────────────────
def normalizar(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def en_arco(theta: float, a_min: Optional[float], a_max: Optional[float]) -> bool:
    """True si theta cae en la zona angular eliminada [a_min, a_max]."""
    if a_min is None or a_max is None:
        return False
    if a_min <= a_max:
        return a_min <= theta <= a_max
    return theta >= a_min or theta <= a_max


def filtrar_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 front_rad: float, rango_max: float,
                 excluir_atras_rad: float = 0.0,
                 rm_min: Optional[float] = None,
                 rm_max: Optional[float] = None):
    """
    /scan crudo → lista de (idx, x, y, af) en marco base_link
    (x adelante, y izquierda), en orden de barrido CONTINUO alrededor del
    robot (ver nota de rotacion abajo).

    - front_rad: ángulo crudo donde apunta el FRENTE del robot (π en Yahboom).
    - excluir_atras_rad: semiancho del cono trasero a descartar (el
      cable/soporte del LiDAR se lee como obstáculo fijo a ~12 cm).
    - rm_min/rm_max: zona angular extra a ignorar (en marco base_link).
    """
    n = len(ranges)
    lim_atras = math.pi - excluir_atras_rad
    # El array crudo del LiDAR "da la vuelta" (indice n-1 -> 0) en
    # raw=angle_min. Con front_rad=180° (Yahboom MS200, cable atras) ese
    # punto de corte cae en af=0 -- el FRENTE -- no atras. pre_segmentar
    # depende de que puntos contiguos en el barrido queden contiguos en
    # esta lista (usa el indice para medir "huecos"); sin rotar, cualquier
    # pared/esquina/caja que el robot tenga justo enfrente se partia en
    # dos clusters en el limite del array, aunque fueran el mismo objeto
    # fisico (esquina real -> se veia como dos clusters cortos en vez de
    # uno con dos lados perpendiculares). Arrancar el barrido "detras" del
    # robot (raw = front_rad + pi) mueve ese corte al cono trasero, que ya
    # se excluye mas abajo -- funciona para cualquier front_rad.
    i0 = int(round((front_rad + math.pi - angle_min) / angle_increment)) % n
    pts = []
    for k in range(n):
        i = (i0 + k) % n
        r = ranges[i]
        if not math.isfinite(r):
            continue
        if r < range_min or r > min(range_max, rango_max):
            continue
        raw = angle_min + i * angle_increment
        af = normalizar(raw - front_rad)
        if abs(af) > lim_atras:            # cono trasero (cable)
            continue
        if en_arco(af, rm_min, rm_max):
            continue
        pts.append((k, r * math.cos(af), r * math.sin(af), af))
    return pts


# ── Clustering (pre-segmentación) ────────────────────────────────────────────
def pre_segmentar(pts_idx, salto_dist: float, salto_idx: int) -> List[List[Point]]:
    """Rompe la nube en clusters por salto euclidiano o hueco de índices."""
    if not pts_idx:
        return []
    grupos, actual = [], [pts_idx[0]]
    for k in range(1, len(pts_idx)):
        ip, xp, yp = pts_idx[k - 1][0], pts_idx[k - 1][1], pts_idx[k - 1][2]
        ic, xc, yc = pts_idx[k][0], pts_idx[k][1], pts_idx[k][2]
        if (ic - ip) > salto_idx or math.hypot(xc - xp, yc - yp) > salto_dist:
            if len(actual) >= 2:
                grupos.append([(p[1], p[2]) for p in actual])
            actual = [pts_idx[k]]
        else:
            actual.append(pts_idx[k])
    if len(actual) >= 2:
        grupos.append([(p[1], p[2]) for p in actual])
    return grupos


# ── Split-and-Merge (IEPF) — corregido ───────────────────────────────────────
def dist_perp(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return math.hypot(px - ax, py - ay)
    return abs(dy * px - dx * py + bx * ay - by * ax) / L


def split(pts: List[Point], umbral: float) -> List[List[Point]]:
    if len(pts) < 2:
        return [pts]
    ax, ay = pts[0]
    bx, by = pts[-1]
    dists = [dist_perp(p[0], p[1], ax, ay, bx, by) for p in pts]
    im = max(range(len(dists)), key=lambda i: dists[i])
    if dists[im] > umbral and len(pts) > 2:
        # SIN [1:]: el punto de quiebre se comparte entre ambos lados.
        # El [1:] de la versión anterior descartaba el primer SUB-GRUPO
        # del lado derecho → una pared de cada esquina desaparecía.
        return split(pts[:im + 1], umbral) + split(pts[im:], umbral)
    return [pts]


def merge(grupos: List[List[Point]], umbral: float) -> List[List[Point]]:
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


def segmentos_de(grupo: List[Point], umbral_split: float, min_pts: int):
    """Cluster → lista de segmentos {p1, p2, lon, ang, mean_y, n}."""
    segs = []
    for s in merge(split(grupo, umbral_split), umbral_split):
        if len(s) < min_pts:
            continue
        p1, p2 = s[0], s[-1]
        lon = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if lon < 0.05:
            continue
        segs.append({
            'p1': p1, 'p2': p2, 'lon': lon,
            'ang': math.atan2(p2[1] - p1[1], p2[0] - p1[0]),
            'mean_y': sum(p[1] for p in s) / len(s),
            'n': len(s),
        })
    return segs


# ── Clasificación ────────────────────────────────────────────────────────────
def _bbox(grupo: List[Point]):
    xs = [p[0] for p in grupo]
    ys = [p[1] for p in grupo]
    return min(xs), max(xs), min(ys), max(ys)


def es_caja_compacta(grupo: List[Point], segs, lado_caja_max: float) -> bool:
    if not segs or len(grupo) < 6:
        return False
    min_x, max_x, min_y, max_y = _bbox(grupo)
    span_x = max_x - min_x
    span_y = max_y - min_y
    diag = math.hypot(span_x, span_y)
    lados = [s['lon'] for s in segs]
    if span_x > lado_caja_max * 1.25 or span_y > lado_caja_max * 1.25:
        return False
    if diag > lado_caja_max * 1.55:
        return False
    return all(0.06 <= l <= lado_caja_max for l in lados)


def clasificar_cluster(grupo, segs, lado_caja_max: float,
                       min_long_pared: float) -> str:
    if not segs:
        return RUIDO
    largos = [s for s in segs if s['lon'] >= min_long_pared]
    if len(largos) >= 2:
        for i in range(len(largos)):
            for j in range(i + 1, len(largos)):
                d = abs(largos[i]['ang'] - largos[j]['ang']) % math.pi
                d = min(d, math.pi - d)
                if abs(d - math.pi / 2) < math.radians(25):
                    return ESQUINA
        return PARED
    if len(largos) == 1:
        return PARED
    if es_caja_compacta(grupo, segs, lado_caja_max):
        return CAJA
    return RUIDO


def analizar_scan(pts_idx, salto_dist, salto_idx, umbral_split,
                  min_puntos, lado_caja_max, min_long_pared):
    """
    Pipeline completo por barrido. Devuelve lista de clusters:
        {'pts', 'segs', 'clase', 'c': centroide (x, y)}
    """
    clusters = []
    for grupo in pre_segmentar(pts_idx, salto_dist, salto_idx):
        if len(grupo) < min_puntos:
            continue
        segs = segmentos_de(grupo, umbral_split, min_puntos)
        clase = clasificar_cluster(grupo, segs, lado_caja_max, min_long_pared)
        cx = sum(p[0] for p in grupo) / len(grupo)
        cy = sum(p[1] for p in grupo) / len(grupo)
        clusters.append({'pts': grupo, 'segs': segs, 'clase': clase,
                         'c': (cx, cy)})
    return clusters


# ── Referencias para el control ──────────────────────────────────────────────
def pared_derecha(clusters, min_long_pared: float, cos_lateral_min: float):
    """
    Pared DERECHA más cercana para el seguimiento: segmento largo,
    mayormente paralelo al avance (|dx|/L > cos_lateral_min) y con
    mean_y < −0.05 (derecha = y negativo en base_link).

    Devuelve {'d': dist. perpendicular LiDAR→pared, 'alpha': ángulo de la
    pared respecto al eje x, en (−π/2, π/2]} o None.
    """
    mejor = None
    for cl in clusters:
        if cl['clase'] not in (PARED, ESQUINA):
            continue
        for s in cl['segs']:
            if s['lon'] < min_long_pared:
                continue
            dx = abs(s['p2'][0] - s['p1'][0])
            if dx / s['lon'] < cos_lateral_min:
                continue                       # pared frontal, no lateral
            if s['mean_y'] >= -0.05:
                continue                       # está a la izquierda
            d = dist_perp(0.0, 0.0, s['p1'][0], s['p1'][1],
                          s['p2'][0], s['p2'][1])
            if mejor is None or d < mejor['d']:
                alpha = s['ang']
                if alpha > math.pi / 2:
                    alpha -= math.pi
                elif alpha <= -math.pi / 2:
                    alpha += math.pi
                mejor = {'d': d, 'alpha': alpha, 'lon': s['lon']}
    return mejor


def caja_derecha(clusters, cos_lateral_min: float):
    """Referencia temporal para seguir la cara de una caja por la derecha."""
    mejor = None
    for cl in clusters:
        if cl['clase'] != CAJA:
            continue
        cx, cy = cl['c']
        if cy >= -0.05 or cx < -0.15 or cx > 0.90:
            continue
        for s in cl['segs']:
            my = 0.5 * (s['p1'][1] + s['p2'][1])
            mx = 0.5 * (s['p1'][0] + s['p2'][0])
            if my >= -0.05 or mx < -0.15 or mx > 0.90:
                continue
            d = dist_perp(0.0, 0.0, s['p1'][0], s['p1'][1],
                          s['p2'][0], s['p2'][1])
            alpha = s['ang']
            if alpha > math.pi / 2:
                alpha -= math.pi
            elif alpha <= -math.pi / 2:
                alpha += math.pi
            dx = abs(s['p2'][0] - s['p1'][0])
            if s['lon'] > 1e-6 and dx / s['lon'] < cos_lateral_min:
                alpha = 0.0
            if mejor is None or d < mejor['d']:
                mejor = {'d': d, 'alpha': alpha, 'lon': s['lon'],
                         'tipo': CAJA}
    return mejor


def frente_y_lados(pts_idx, clusters, off_frente, off_lado,
                   margen_lateral: float = 0.06):
    """
    Distancias de seguridad medidas al BORDE del robot (post-offset):
      d_frente : espacio libre real por delante dentro del corredor de
                 colisión (ancho real del robot + margen)
      clase_frente : clase del cluster dueño del punto más cercano al frente
      d_izq / d_der : espacio libre lateral (franja a la altura del robot)
      punto_footprint : punto que invade el footprint inflado, o None
    """
    semi = off_lado + margen_lateral
    d_min, clase_f = float('inf'), None
    # frente: punto más cercano dentro del corredor, con su cluster
    for cl in clusters:
        for (x, y) in cl['pts']:
            if x > 0.0 and abs(y) <= semi and x < d_min:
                d_min, clase_f = x, cl['clase']
    d_frente = d_min - off_frente

    d_izq = d_der = float('inf')
    punto_fp = None
    for p in pts_idx:
        x, y = p[1], p[2]
        # franja lateral a la altura del cuerpo del robot
        if -0.10 <= x <= off_frente + 0.05:
            if y > 0:
                d_izq = min(d_izq, y - off_lado)
            else:
                d_der = min(d_der, -y - off_lado)
        # footprint inflado (emergencia)
        if punto_fp is None and (-0.10 - 0.02 <= x <= off_frente + 0.03
                                 and abs(y) <= off_lado + 0.03):
            punto_fp = (x, y)
    return d_frente, clase_f, d_izq, d_der, punto_fp
