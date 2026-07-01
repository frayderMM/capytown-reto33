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
