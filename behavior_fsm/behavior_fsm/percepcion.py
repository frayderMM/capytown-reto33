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

3. CLASIFICACIÓN POR TAMAÑO (regla del reto, simplificada):
       CAJA  → el cluster completo cabe en el tamaño de una caja real
               (lados ≤ lado_caja_max, ~20 cm + tol; diagonal máx. 28 cm)
       PARED → el cluster es más grande que eso (sea pared recta o
               esquina en L — el FSM trata ambas igual, así que no se
               distinguen para evitar ruido de clasificación cerca del
               umbral)
       RUIDO → cluster sin segmentos válidos
"""

import math
import random
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


def _cluster_mayor_que_caja(grupo: List[Point], lado_caja_max: float) -> bool:
    """True si el cluster ya no cabe en el tamaño de una caja real (20 cm)."""
    min_x, max_x, min_y, max_y = _bbox(grupo)
    span_x = max_x - min_x
    span_y = max_y - min_y
    diag = math.hypot(span_x, span_y)
    return (span_x > lado_caja_max * 1.25 or span_y > lado_caja_max * 1.25
            or diag > lado_caja_max * 1.55)


def es_caja_compacta(grupo: List[Point], segs, lado_caja_max: float) -> bool:
    if not segs or len(grupo) < 6:
        return False
    if _cluster_mayor_que_caja(grupo, lado_caja_max):
        return False
    lados = [s['lon'] for s in segs]
    return all(0.06 <= l <= lado_caja_max for l in lados)


def clasificar_cluster(grupo, segs, lado_caja_max: float) -> str:
    """
    Regla única por tamaño: el cluster completo cabe en una caja real
    (~20 cm, con tolerancia) -> CAJA; si es más grande -> PARED. No importa
    si está al frente o al costado ni si forma una L (esquina) — el FSM ya
    trata PARED y ESQUINA igual, así que separarlas solo agregaba ruido de
    clasificación cerca del umbral.
    """
    if not segs:
        return RUIDO
    if _cluster_mayor_que_caja(grupo, lado_caja_max):
        return PARED
    if es_caja_compacta(grupo, segs, lado_caja_max):
        return CAJA
    return RUIDO


def analizar_scan(pts_idx, salto_dist, salto_idx, umbral_split,
                  min_puntos, lado_caja_max):
    """
    Pipeline completo por barrido. Devuelve lista de clusters:
        {'pts', 'segs', 'clase', 'c': centroide (x, y)}
    """
    clusters = []
    for grupo in pre_segmentar(pts_idx, salto_dist, salto_idx):
        if len(grupo) < min_puntos:
            continue
        segs = segmentos_de(grupo, umbral_split, min_puntos)
        clase = clasificar_cluster(grupo, segs, lado_caja_max)
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


def _mediana_cercanos(vals, n=6):
    if not vals:
        return None
    vals = sorted(vals)[:n]
    m = len(vals) // 2
    return vals[m] if len(vals) % 2 else 0.5 * (vals[m - 1] + vals[m])


def camino_derecho(pts_idx, off_lado: float):
    """
    Borde derecho por puntos frontales del LiDAR. La decision de avance y
    correccion usa solo lo que el robot ve hacia adelante; la parte trasera se
    reserva para seguridad anti-choque en frente_y_lados().
    """
    front = []
    center = []
    for p in pts_idx:
        x, y = p[1], p[2]
        if y >= -(off_lado + 0.015) or y < -0.55:
            continue
        d = -y
        if 0.12 <= x <= 0.65:
            front.append(d)
        elif 0.00 <= x <= 0.30:
            center.append(d)

    df = _mediana_cercanos(front)
    dc = _mediana_cercanos(center)

    if df is not None:
        alpha = 0.0 if dc is None else math.atan2(dc - df, 0.30)
        return {'d': df, 'alpha': alpha, 'lon': 0.35,
                'tipo': 'CAMINO_DER_FRENTE',
                'd_front': df, 'd_center': dc}
    if dc is not None:
        return {'d': dc, 'alpha': 0.0, 'lon': 0.20,
                'tipo': 'CAMINO_DER_FRENTE',
                'd_front': dc, 'd_center': dc}
    return None


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
      punto_footprint : punto frontal que invade el footprint, o None
      punto_trasero : punto trasero/costado posterior demasiado cerca, o None
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
    punto_trasero = None
    for p in pts_idx:
        x, y = p[1], p[2]
        # franja lateral a la altura del cuerpo del robot
        if -0.10 <= x <= off_frente + 0.05:
            if y > 0:
                d_izq = min(d_izq, y - off_lado)
            else:
                d_der = min(d_der, -y - off_lado)
        # Frenado de seguridad solo por invasión frontal real. La pared
        # derecha puede pasar cerca del costado sin ser choque; si se toma
        # como footprint, el robot se frena innecesariamente sin avanzar.
        if punto_fp is None and (0.02 <= x <= off_frente + 0.04
                                 and abs(y) <= off_lado * 0.80):
            punto_fp = (x, y)
        if punto_trasero is None and (-0.16 <= x <= -0.02
                                      and abs(y) <= off_lado + 0.035):
            punto_trasero = (x, y)
    return d_frente, clase_f, d_izq, d_der, punto_fp, punto_trasero


# ── RANSAC para pared lateral (wall_follower) ────────────────────────────────
# Alternativa a Split-and-Merge para el seguimiento lateral: no requiere que
# los puntos de la pared sean contiguos, así que si una caja interrumpe el
# tramo que ve el LiDAR, sus puntos quedan fuera del consenso (outliers) en
# vez de cortar o fusionar mal el segmento.
Recta = Tuple[float, float, float]  # a*x + b*y + c = 0, normalizada (a²+b²=1)


def polar_a_cartesiano(angulo: float, rango: float) -> Point:
    return rango * math.cos(angulo), rango * math.sin(angulo)


def recta_por_pca(puntos_xy: List[Point]) -> Recta:
    """Ajusta una recta por regresión ortogonal (PCA): la normal es el
    autovector de MENOR varianza, así no degenera con rectas casi verticales
    como sí lo haría una regresión y=mx+b."""
    n = len(puntos_xy)
    mx = sum(p[0] for p in puntos_xy) / n
    my = sum(p[1] for p in puntos_xy) / n
    sxx = sum((p[0] - mx) ** 2 for p in puntos_xy)
    syy = sum((p[1] - my) ** 2 for p in puntos_xy)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in puntos_xy)

    if abs(sxx - syy) < 1e-12 and abs(sxy) < 1e-12:
        theta = 0.0
    else:
        theta = 0.5 * math.atan2(2 * sxy, sxx - syy)

    def residual(a, b):
        return sum((a * (p[0] - mx) + b * (p[1] - my)) ** 2 for p in puntos_xy)

    cand_a = (-math.sin(theta), math.cos(theta))
    cand_b = (math.cos(theta), math.sin(theta))
    a, b = cand_a if residual(*cand_a) <= residual(*cand_b) else cand_b
    c = -(a * mx + b * my)
    return a, b, c


def distancia_recta_origen(recta: Recta) -> float:
    """Distancia perpendicular del origen (el robot, marco base_link) a la recta."""
    _, _, c = recta
    return abs(c)


def ajustar_recta_ransac(puntos_xy: List[Point], umbral_inlier: float = 0.03,
                         iteraciones: int = 80, min_inliers: int = 12,
                         rng: Optional[random.Random] = None) -> Optional[dict]:
    """RANSAC: muestrea 2 puntos al azar por iteración, cuenta inliers a la
    recta que forman, y al final refina con recta_por_pca() sobre el mejor
    conjunto de inliers. Devuelve {'a','b','c','inliers','ratio'} o None."""
    n = len(puntos_xy)
    if n < min_inliers:
        return None
    if rng is None:
        rng = random.Random()

    mejor_inliers: List[int] = []
    for _ in range(iteraciones):
        i, j = rng.sample(range(n), 2)
        x1, y1 = puntos_xy[i]
        x2, y2 = puntos_xy[j]
        dx, dy = x2 - x1, y2 - y1
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            continue
        a, b = -dy / norm, dx / norm
        c = -(a * x1 + b * y1)

        inliers = [k for k, (px, py) in enumerate(puntos_xy)
                  if abs(a * px + b * py + c) <= umbral_inlier]
        if len(inliers) > len(mejor_inliers):
            mejor_inliers = inliers

    if len(mejor_inliers) < min_inliers:
        return None

    pts_inlier = [puntos_xy[k] for k in mejor_inliers]
    a, b, c = recta_por_pca(pts_inlier)
    return {'a': a, 'b': b, 'c': c, 'inliers': mejor_inliers,
            'ratio': len(mejor_inliers) / n}
