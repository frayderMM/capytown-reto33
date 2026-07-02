#!/usr/bin/env python3
r"""
wall_follower.py  —  Alineacion paralela con la pared DERECHA (metodo de 2 haces).

Suscribe a /scan, toma dos lecturas del lado derecho (una casi perpendicular
y otra un poco adelantada hacia el frente) y calcula el angulo entre el
robot y la pared. Publica esa correccion angular en /lateral_correction
(std_msgs/Float32) para que behavior_fsm la aplique en CRUCERO (avanza con
vel_crucero mientras se autoalinea con esta correccion).

Geometria (vista superior, el robot avanza hacia arriba):

              r_frente
                 \
                  \  theta
    -------------- x ----- pared derecha (y < 0)
                  |
                  | r_derecha (perpendicular, angulo_derecha_deg)

    alpha = atan2( r_frente*cos(theta) - r_derecha , r_frente*sin(theta) )

    alpha == 0   -> el robot esta paralelo a la pared.
    alpha != 0   -> hay que girar hasta que alpha vuelva a 0.

    Esto se recalcula en CADA mensaje de /scan: no es una calibracion de
    una sola vez, es continua mientras el nodo esta vivo (se autocorrige
    todo el rato, tambien mientras el robot avanza).

    El signo de la correccion (que lado hay que girar) se ajusta con el
    parametro 'signo_correccion' (+1.0 o -1.0): en la primera prueba ESTATICA
    (robot quieto, girado a mano junto a la pared) hay que confirmar que el
    signo de 'w' publicado tenga sentido antes de dejar que el robot se mueva
    solo. Si gira para el lado contrario, cambiar signo_correccion a -1.0.

    Con |alpha| por debajo de 'umbral_paralelo_deg' se considera "ya esta
    paralelo" y no se publica correccion (evita vibrar cerca de cero).
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # ── Parametros ────────────────────────────────────────────────────
        # Angulos en convencion ROS: 0=frente, +90=izquierda, -90=derecha.
        self.declare_parameter('angulo_frente_deg', -50.0)   # haz adelantado (entre frente y derecha)
        self.declare_parameter('angulo_derecha_deg', -90.0)  # haz perpendicular a la derecha
        self.declare_parameter('ventana_busqueda_deg', 3.0)  # +/- grados para buscar lectura valida
        self.declare_parameter('Kp', 1.0)                    # ganancia: alpha (rad) -> w (rad/s)
        self.declare_parameter('signo_correccion', 1.0)      # +1.0 o -1.0, se ajusta en prueba estatica
        self.declare_parameter('max_correccion', 0.40)       # rad/s - saturacion de la salida
        self.declare_parameter('umbral_paralelo_deg', 3.0)   # deg - por debajo, se considera paralelo
        self.declare_parameter('log_cada_seg', 0.5)          # s - periodo de log de estado

        self._ang_frente = math.radians(self.get_parameter('angulo_frente_deg').value)
        self._ang_derecha = math.radians(self.get_parameter('angulo_derecha_deg').value)
        self._ventana = math.radians(self.get_parameter('ventana_busqueda_deg').value)
        self._Kp = self.get_parameter('Kp').value
        self._signo = self.get_parameter('signo_correccion').value
        self._max_w = self.get_parameter('max_correccion').value
        self._umbral_deg = self.get_parameter('umbral_paralelo_deg').value
        self._log_periodo = self.get_parameter('log_cada_seg').value

        self._theta = abs(self._ang_derecha - self._ang_frente)
        self._t_ultimo_log = self.get_clock().now()

        # ── ROS I/O ───────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)
        self._pub = self.create_publisher(Float32, '/lateral_correction', 10)

        self.get_logger().info(
            f'wall_follower listo (paralelo a pared DERECHA)  |  '
            f'ang_frente={math.degrees(self._ang_frente):.0f} deg  '
            f'ang_derecha={math.degrees(self._ang_derecha):.0f} deg  '
            f'Kp={self._Kp}  signo={self._signo}')

    # ── Lectura robusta de un haz por angulo objetivo ──────────────────────
    def _rango_en_angulo(self, msg: LaserScan, angulo_obj: float):
        """Busca, cerca de 'angulo_obj', la lectura valida mas cercana a ese angulo."""
        idx_obj = int(round((angulo_obj - msg.angle_min) / msg.angle_increment))
        medio = max(1, int(round(self._ventana / msg.angle_increment)))
        mejor_r = None
        mejor_di = None
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

    # ── Log de estado, throttled ────────────────────────────────────────────
    def _log_estado(self, texto: str):
        ahora = self.get_clock().now()
        if (ahora - self._t_ultimo_log).nanoseconds * 1e-9 >= self._log_periodo:
            self.get_logger().info(texto)
            self._t_ultimo_log = ahora

    # ── Callback principal ────────────────────────────────────────────────
    def _cb_scan(self, msg: LaserScan):
        r_frente = self._rango_en_angulo(msg, self._ang_frente)
        r_derecha = self._rango_en_angulo(msg, self._ang_derecha)

        if r_frente is None or r_derecha is None:
            self._log_estado('SIN LECTURA valida en el lado derecho (revisar orientacion del LiDAR)')
            return

        alpha = math.atan2(r_frente * math.cos(self._theta) - r_derecha,
                           r_frente * math.sin(self._theta))
        alpha_deg = math.degrees(alpha)
        dist_pared = r_derecha * math.cos(alpha)

        if abs(alpha_deg) <= self._umbral_deg:
            w = 0.0
            estado = 'PARALELO'
        else:
            w = self._signo * self._Kp * alpha
            w = max(-self._max_w, min(self._max_w, w))
            estado = 'gira IZQ (w>0)' if w > 0 else 'gira DER (w<0)'

        out = Float32()
        out.data = float(w)
        self._pub.publish(out)

        self._log_estado(
            f'{estado:16s}  alpha={alpha_deg:+6.1f} deg  '
            f'r_frente={r_frente:.2f}  r_derecha={r_derecha:.2f}  '
            f'dist_pared={dist_pared:.2f} m  w={w:+.3f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    nodo = WallFollower()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
