"""
lidar_utils.py
--------------
Funciones auxiliares para el procesamiento de un LiDAR 2D (sensor_msgs/LaserScan).

Reto CapyTown - "El Censo y el Guardian de las Cajas"
ESAN - Robotica de Moviles 2026-I

Estas funciones son PURAS (sin estado, sin ROS) para poder probarlas
de forma independiente y reutilizarlas tanto en el detector como en la FSM.
"""

import math
from typing import List, Tuple

# Un "punto" del barrido se representa como (angulo, rango) en coordenadas polares.
PuntoPolar = Tuple[float, float]
PuntoXY = Tuple[float, float]
# Una recta se representa como (a, b, c) normalizada (a^2+b^2=1) de la forma
# a*x + b*y + c = 0. La distancia de cualquier punto (px,py) a la recta es
# |a*px + b*py + c|; en particular la distancia del origen (el robot, en
# marco base_link) a la recta es simplemente |c|.
Recta = Tuple[float, float, float]


def filtrar_scan(ranges: List[float],
                 angle_min: float,
                 angle_increment: float,
                 range_min: float,
                 range_max: float) -> List[PuntoPolar]:
    """Convierte el arreglo crudo de rangos en una lista de puntos validos.

    Descarta lecturas inf/nan y las que caen fuera del rango fisico del sensor.

    Devuelve una lista de tuplas (angulo, rango) en orden de barrido.
    """
    puntos: List[PuntoPolar] = []
    for i, r in enumerate(ranges):
        # math.isfinite() descarta inf y nan de una sola vez.
        if not math.isfinite(r):
            continue
        if r < range_min or r > range_max:
            continue
        angulo = angle_min + i * angle_increment
        puntos.append((angulo, r))
    return puntos


def polar_a_cartesiano(angulo: float, rango: float) -> Tuple[float, float]:
    """Pasa de (angulo, rango) en el marco del robot a (x, y) en el marco del robot.

    Convencion ROS: x hacia adelante, y hacia la izquierda, angulo CCW.
    """
    x = rango * math.cos(angulo)
    y = rango * math.sin(angulo)
    return x, y


def componer_odom(x_robot: float, y_robot: float,
                  pose_x: float, pose_y: float, pose_yaw: float
                  ) -> Tuple[float, float]:
    """Transforma un punto del marco del robot (base) al marco 'odom'.

    Aplica la composicion T_odom_base: rotacion por el yaw del robot mas
    traslacion por la posicion del robot en odom.

        [x_odom]   [cos(yaw)  -sin(yaw)] [x_robot]   [pose_x]
        [y_odom] = [sin(yaw)   cos(yaw)] [y_robot] + [pose_y]
    """
    c = math.cos(pose_yaw)
    s = math.sin(pose_yaw)
    x_odom = c * x_robot - s * y_robot + pose_x
    y_odom = s * x_robot + c * y_robot + pose_y
    return x_odom, y_odom


def yaw_desde_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extrae el yaw (rotacion en Z) de un quaternion. Util para leer /odom."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clustering_1d(puntos: List[PuntoPolar],
                  umbral_salto: float) -> List[List[PuntoPolar]]:
    """Agrupa puntos consecutivos del barrido por discontinuidad de rango.

    Recorre los puntos en orden angular. Si el salto de rango entre un punto
    y el siguiente supera 'umbral_salto', se cierra el cluster actual y se
    inicia uno nuevo. Es el clustering 1D clasico para LiDAR de un solo plano.
    """
    if not puntos:
        return []

    clusters: List[List[PuntoPolar]] = []
    cluster_actual: List[PuntoPolar] = [puntos[0]]

    for i in range(1, len(puntos)):
        r_prev = puntos[i - 1][1]
        r_act = puntos[i][1]
        if abs(r_act - r_prev) > umbral_salto:
            clusters.append(cluster_actual)
            cluster_actual = [puntos[i]]
        else:
            cluster_actual.append(puntos[i])

    clusters.append(cluster_actual)
    return clusters


def centroide_cluster(cluster: List[PuntoPolar]) -> Tuple[float, float]:
    """Centroide del cluster en coordenadas cartesianas del marco del robot.

    Promedia los puntos ya convertidos a (x, y) -- mas estable que promediar
    angulo y rango por separado.
    """
    sx = 0.0
    sy = 0.0
    for ang, r in cluster:
        x, y = polar_a_cartesiano(ang, r)
        sx += x
        sy += y
    n = len(cluster)
    return sx / n, sy / n


def ancho_cluster(cluster: List[PuntoPolar]) -> float:
    """Ancho aparente del cluster: distancia entre su primer y ultimo punto."""
    if len(cluster) < 2:
        return 0.0
    x0, y0 = polar_a_cartesiano(*cluster[0])
    x1, y1 = polar_a_cartesiano(*cluster[-1])
    return math.hypot(x1 - x0, y1 - y0)


def distancia(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Distancia euclidiana entre dos puntos (x, y)."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def puntos_a_xy(puntos: List[PuntoPolar]) -> List[PuntoXY]:
    """Convierte una lista de (angulo, rango) a (x, y) en el marco del robot."""
    return [polar_a_cartesiano(ang, r) for ang, r in puntos]


def recta_por_pca(puntos_xy: List[PuntoXY]) -> Recta:
    """Ajusta una recta por regresion ortogonal (PCA) a un conjunto de puntos.

    A diferencia de una regresion y=mx+b, no degenera con rectas casi
    verticales: la normal de la recta es el autovector de MENOR varianza de
    la matriz de covarianza 2D de los puntos. Se prueban las dos direcciones
    ortogonales candidatas (los dos autovectores) y se elige la que de menor
    suma de residuales al cuadrado, evitando depender del signo/convencion
    de la formula del angulo principal.

    Devuelve (a, b, c) normalizada tal que a*x + b*y + c = 0.
    """
    n = len(puntos_xy)
    mx = sum(p[0] for p in puntos_xy) / n
    my = sum(p[1] for p in puntos_xy) / n
    sxx = sum((p[0] - mx) ** 2 for p in puntos_xy)
    syy = sum((p[1] - my) ** 2 for p in puntos_xy)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in puntos_xy)

    if abs(sxx - syy) < 1e-12 and abs(sxy) < 1e-12:
        theta = 0.0  # nube ~circular (sin puntos suficientes); normal arbitraria
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


# ── Split-and-Merge ──────────────────────────────────────────────────────
# Segmenta una nube de puntos en tramos localmente rectos. Se probo RANSAC
# (consenso robusto a outliers) pero en la practica dio peor resultado: el
# muestreo aleatorio produce variacion de cuadro a cuadro incluso con la
# pared quieta (ruido en el control, y sobre todo en esquinas eleccion de
# lado inconsistente entre intentos identicos). Split-and-Merge separa la
# pared de una caja que la interrumpe de forma DETERMINISTICA (mismo scan
# -> mismo resultado siempre), y luego se elige el mejor segmento para el
# ajuste final por minimos cuadrados.

def pre_segmentar(puntos_idx_xy: List[Tuple[int, float, float]],
                  salto_dist: float, salto_idx: int) -> List[List[PuntoXY]]:
    """Rompe una nube de puntos (idx_barrido, x, y) ordenada en grupos
    contiguos, cortando donde hay un hueco de indice (puntos no
    consecutivos en el barrido, p.ej. por lecturas invalidas filtradas) o
    un salto euclidiano grande (discontinuidad de objeto: el borde de una
    caja frente a la pared). Devuelve una lista de grupos, cada uno una
    lista de (x, y). Descarta grupos de un solo punto (no definen recta).
    """
    if not puntos_idx_xy:
        return []
    grupos: List[List[PuntoXY]] = []
    actual = [puntos_idx_xy[0]]
    for k in range(1, len(puntos_idx_xy)):
        ip, xp, yp = puntos_idx_xy[k - 1]
        ic, xc, yc = puntos_idx_xy[k]
        if (ic - ip) > salto_idx or math.hypot(xc - xp, yc - yp) > salto_dist:
            if len(actual) >= 2:
                grupos.append([(x, y) for _, x, y in actual])
            actual = [puntos_idx_xy[k]]
        else:
            actual.append(puntos_idx_xy[k])
    if len(actual) >= 2:
        grupos.append([(x, y) for _, x, y in actual])
    return grupos


def _distancia_perpendicular_segmento(px, py, ax, ay, bx, by) -> float:
    """Distancia de (px,py) a la recta que pasa por (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return math.hypot(px - ax, py - ay)
    return abs(dy * px - dx * py + bx * ay - by * ax) / L


def dividir_split(puntos_xy: List[PuntoXY], umbral: float) -> List[List[PuntoXY]]:
    """Paso 'split': si el punto de mayor desviacion perpendicular a la
    cuerda (primer punto - ultimo punto) supera el umbral, corta ahi y
    repite recursivamente en cada mitad. Cuando ya no hay que cortar mas,
    devuelve el tramo tal cual (localmente recto dentro del umbral)."""
    if len(puntos_xy) < 3:
        return [puntos_xy]
    ax, ay = puntos_xy[0]
    bx, by = puntos_xy[-1]
    dists = [_distancia_perpendicular_segmento(p[0], p[1], ax, ay, bx, by)
             for p in puntos_xy]
    im = max(range(len(dists)), key=lambda i: dists[i])
    if dists[im] > umbral:
        return dividir_split(puntos_xy[:im + 1], umbral) + dividir_split(puntos_xy[im:], umbral)[1:]
    return [puntos_xy]


def fusionar_merge(segmentos: List[List[PuntoXY]], umbral: float) -> List[List[PuntoXY]]:
    """Paso 'merge': fusiona segmentos adyacentes si, juntos, siguen
    siendo una sola recta dentro del umbral — evita sobre-segmentar un
    tramo recto que el split partio de mas por ruido puntual."""
    if len(segmentos) <= 1:
        return segmentos
    fusionados = [segmentos[0]]
    for seg in segmentos[1:]:
        candidato = fusionados[-1] + seg
        if len(dividir_split(candidato, umbral)) == 1:
            fusionados[-1] = candidato
        else:
            fusionados.append(seg)
    return fusionados


def segmentar_split_and_merge(puntos_idx_xy: List[Tuple[int, float, float]],
                              umbral_split: float,
                              salto_dist: float,
                              salto_idx: int,
                              min_puntos_segmento: int = 4) -> List[List[PuntoXY]]:
    """Pipeline completo: pre-segmenta por discontinuidad de objeto, luego
    aplica split+merge a cada grupo contiguo. Descarta segmentos con menos
    de min_puntos_segmento puntos (ruido, no una pared).

    Devuelve la lista final de segmentos, cada uno una lista de (x, y).
    """
    segmentos_finales: List[List[PuntoXY]] = []
    for grupo in pre_segmentar(puntos_idx_xy, salto_dist, salto_idx):
        for seg in fusionar_merge(dividir_split(grupo, umbral_split), umbral_split):
            if len(seg) >= min_puntos_segmento:
                segmentos_finales.append(seg)
    return segmentos_finales


def largo_segmento(segmento: List[PuntoXY]) -> float:
    """Longitud de un segmento: distancia entre su primer y ultimo punto."""
    if len(segmento) < 2:
        return 0.0
    (x0, y0), (x1, y1) = segmento[0], segmento[-1]
    return math.hypot(x1 - x0, y1 - y0)


def es_segmento_lateral(segmento: List[PuntoXY], cos_min: float = 0.75) -> bool:
    """True si el segmento va mayormente a lo LARGO del eje x (direccion
    de avance del robot) en vez de CRUZADO (como una pared frontal en una
    esquina, o el tramo donde el pasillo dobla).

    Mejora sobre el diseño anterior: en una esquina, antes de que el robot
    termine de girar, un tramo de la pared que esta justo doblando puede
    colarse en el sector lateral (izq/der) y ser lo bastante largo para
    pasar el filtro de longitud — pero esa pared no es paralela al
    pasillo, es casi perpendicular. Este filtro la descarta como
    candidata a "pared lateral" aunque sea larga, evitando que una lectura
    de la esquina se confunda con la pared del costado.

    cos_min: coseno minimo del angulo con el eje x (0.75 ~ 41° de
    tolerancia); mas alto = mas estricto (exige mas paralelismo).
    """
    (x0, y0), (x1, y1) = segmento[0], segmento[-1]
    dx, dy = x1 - x0, y1 - y0
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return False
    return abs(dx) / L >= cos_min
