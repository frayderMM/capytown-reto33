#!/usr/bin/env python3
"""
behavior_fsm.py  —  Guardian v4

Estados: CRUCERO → GIRO → RODEO → CRUCERO

dist_frente y dist_izq_raw/dist_der_raw (minimo crudo, sin filtrar) se
miden localmente sobre /scan: son la base del STOP de seguridad y deben
ver CUALQUIER objeto cercano. dist_izq / dist_der llegan del nodo
wall_follower (RANSAC sobre las paredes laterales, via /dist_izq y
/dist_der) y se usan solo para el tracking de crucero y la eleccion de
lado en GIRO — RANSAC descarta a proposito los objetos que no son pared
(p.ej. una caja pegada al costado) como outliers, por lo que NO sirven
para detectar un choque lateral inminente.

CRUCERO:  sigue pared derecha.
          f >= d_front_ini → solo w_der (tracking pared)
          f <  d_front_ini → solo w_front (evasion frontal, w_der=0)
          f <= d_giro      → pasa a GIRO, eligiendo lado y guardando el
                             ancho de jiron disponible en ese instante.
GIRO:     gira hacia el lado con MAS espacio (compara dist_izq vs dist_der
          al momento de entrar), con magnitud proporcional a que tan cerca
          esta el obstaculo (mas cerca → giro mas cerrado).
          Sale si: izq<d_izq_salida | frente despeja tras t_giro_min | max t_giro_max
RODEO:    avanza RECTO (w=0) hasta que el frente vuelve a despejarse
          (sensor, no temporizador), con un minimo y un maximo de seguridad.
CRUCERO(recovery): durante t_recuperacion, solo w_der sin w_front.

Topicos debug: /dist_frente /dbg/dist_izq_fsm /dbg/dist_der_fsm
               /dbg/w_front /dbg/w_der /dbg/w_izq /dbg/w_total
Suscritos:     /dist_izq /dist_der (de wall_follower, RANSAC)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


CRUCERO = 'CRUCERO'
GIRO    = 'GIRO'
RODEO   = 'RODEO'


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  30.0)
        self.declare_parameter('t_espera_inicio',     10.0)  # s  cuenta regresiva tras ENTER

        self.declare_parameter('d_stop_front',   0.14)
        self.declare_parameter('d_stop_lateral', 0.06)
        self.declare_parameter('d_giro',         0.30)
        self.declare_parameter('d_front_inicio', 0.40)

        self.declare_parameter('target_der', 0.17)   # robot consistente a 17-20cm
        self.declare_parameter('Kder',        2.6)   # ganancia proporcional (subida de 2.0: pegaba poco)
        self.declare_parameter('Kd_der',      0.3)   # derivativo: amortigua el overshoot de subir Kder
        self.declare_parameter('d_izq_min',  0.15)
        self.declare_parameter('Kizq',        4.0)
        self.declare_parameter('Kfront',      2.0)

        self.declare_parameter('vel_crucero',      0.10)
        self.declare_parameter('vel_maniobra',     0.05)   # lineal en GIRO/RODEO: bajado de
                                                            # 0.07 -> 0.05, el robot pasaba bien
                                                            # pero avanzaba mas de lo necesario
                                                            # haciendo el giro/rodeo
        self.declare_parameter('vel_giro_gradual', 0.55)
        self.declare_parameter('max_w',            0.60)

        self.declare_parameter('t_giro_min',        0.8)
        self.declare_parameter('t_giro_max',        4.0)
        self.declare_parameter('d_lado_salida_giro', 0.20)  # sale de GIRO si el lado hacia el que gira se cierra
        self.declare_parameter('k_urgencia_giro',   1.7)   # amplifica w_giro si el obstaculo esta muy cerca
        self.declare_parameter('t_rodeo_min',       0.4)   # minimo recto tras GIRO antes de evaluar salida
        self.declare_parameter('t_rodeo_max',       1.0)   # salvavidas — RODEO es ciego (w=0)
        self.declare_parameter('t_cooldown',        2.0)
        self.declare_parameter('t_recuperacion',    1.5)

        # ── Cargar ────────────────────────────────────────────────────────
        self.front_rad = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector    = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.t_espera_inicio = self.get_parameter('t_espera_inicio').value
        self._reload_params()
        self.add_on_set_parameters_callback(self._on_params)

        # ── Estado ────────────────────────────────────────────────────────
        self.estado        = CRUCERO
        self.t_inicio      = self.get_clock().now()
        self.t_ultimo_giro = -float('inf')
        self.dir_giro      = 1.0   # +1 = izquierda (w>0), -1 = derecha (w<0)
        self.w_giro_efectivo = 0.0  # magnitud del giro, congelada al entrar a GIRO

        # ── PD tracking pared derecha ────────────────────────────────────
        self._err_der_prev = 0.0
        self._t_der_prev   = self.get_clock().now()

        # ── Sensores ──────────────────────────────────────────────────────
        self.dist_frente = float('inf')
        # dist_izq / dist_der llegan de wall_follower (RANSAC): sirven para el
        # tracking de crucero y la eleccion de lado en GIRO, pero RANSAC
        # descarta a proposito los puntos que no son pared (p.ej. una caja
        # pegada al costado) como outliers. Por eso el STOP de seguridad
        # lateral NO puede usar estos valores: usa dist_izq_raw/dist_der_raw
        # (minimo crudo, sin filtrar, calculado aqui mismo como el frente).
        self.dist_izq    = float('inf')
        self.dist_der    = float('inf')
        self.dist_izq_raw = float('inf')
        self.dist_der_raw = float('inf')

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.cb_scan, _qos)
        self.create_subscription(Float32, '/dist_izq', self._cb_dist_izq, _qos)
        self.create_subscription(Float32, '/dist_der', self._cb_dist_der, _qos)
        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel',   10)
        self.pub_estado = self.create_publisher(String, '/fsm_state', 10)

        # /dist_izq y /dist_der los publica wall_follower (RANSAC); este nodo
        # solo los consume. Republicamos lo que efectivamente usa la FSM bajo
        # /dbg/ para no crear un segundo publicador sobre el mismo topico.
        self.pub_df  = self.create_publisher(Float32, '/dist_frente',  10)
        self.pub_dl  = self.create_publisher(Float32, '/dbg/dist_izq_fsm', 10)
        self.pub_dr  = self.create_publisher(Float32, '/dbg/dist_der_fsm', 10)
        self.pub_wf  = self.create_publisher(Float32, '/dbg/w_front',  10)
        self.pub_wd  = self.create_publisher(Float32, '/dbg/w_der',    10)
        self.pub_wi  = self.create_publisher(Float32, '/dbg/w_izq',    10)
        self.pub_wt  = self.create_publisher(Float32, '/dbg/w_total',  10)

        self.create_timer(0.05, self.loop_control)

        self.get_logger().info(
            f'Guardian v4  giro<{self.d_giro}m  front_ini={self.d_front_ini}m'
            f'  target_der={self.target_der}m  Kder={self.Kder}'
            f'  t_rodeo=[{self.t_rodeo_min},{self.t_rodeo_max}]s (salida por sensor)')

    def _reload_params(self):
        self.d_stop_front      = self.get_parameter('d_stop_front').value
        self.d_stop_lat        = self.get_parameter('d_stop_lateral').value
        self.d_giro            = self.get_parameter('d_giro').value
        self.d_front_ini       = self.get_parameter('d_front_inicio').value
        self.target_der        = self.get_parameter('target_der').value
        self.Kder               = self.get_parameter('Kder').value
        self.Kd_der             = self.get_parameter('Kd_der').value
        self.d_izq_min         = self.get_parameter('d_izq_min').value
        self.Kizq              = self.get_parameter('Kizq').value
        self.Kfront            = self.get_parameter('Kfront').value
        self.v_cruise          = self.get_parameter('vel_crucero').value
        self.v_maniobra        = self.get_parameter('vel_maniobra').value
        self.w_giro            = self.get_parameter('vel_giro_gradual').value
        self.max_w             = self.get_parameter('max_w').value
        self.t_giro_min        = self.get_parameter('t_giro_min').value
        self.t_giro_max        = self.get_parameter('t_giro_max').value
        self.d_lado_salida_giro = self.get_parameter('d_lado_salida_giro').value
        self.k_urgencia_giro   = self.get_parameter('k_urgencia_giro').value
        self.t_rodeo_min       = self.get_parameter('t_rodeo_min').value
        self.t_rodeo_max       = self.get_parameter('t_rodeo_max').value
        self.t_cooldown        = self.get_parameter('t_cooldown').value
        self.t_recuperacion    = self.get_parameter('t_recuperacion').value

    def _on_params(self, params):
        self._reload_params()
        self.get_logger().info(f'Params: {[p.name for p in params]}')
        return SetParametersResult(successful=True)

    def cb_scan(self, msg: LaserScan):
        # Frente + minimo crudo lateral: se miden aqui mismo, sin depender de
        # otro nodo ni de RANSAC, porque son la base del STOP de seguridad y
        # necesitan ver CUALQUIER objeto cercano (pared o caja), no solo la
        # pared "limpia" que RANSAC reporta para el tracking de crucero.
        d_f = float('inf')
        d_l_raw = d_r_raw = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            abs_af = abs(af)
            if abs_af <= self.sector:
                d_f = min(d_f, r)
            elif af > 0:
                d_l_raw = min(d_l_raw, r)
            else:
                d_r_raw = min(d_r_raw, r)
        self.dist_frente   = d_f
        self.dist_izq_raw  = d_l_raw
        self.dist_der_raw  = d_r_raw

    def _cb_dist_izq(self, msg: Float32):
        self.dist_izq = msg.data

    def _cb_dist_der(self, msg: Float32):
        self.dist_der = msg.data

    def _pub(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String(); s.data = self.estado
        self.pub_estado.publish(s)

    def _pub_dbg(self, wf, wd, wi, wt):
        def f32(x):
            m = Float32()
            m.data = float(x) if math.isfinite(x) else -999.0
            return m
        self.pub_df.publish(f32(self.dist_frente))
        self.pub_dl.publish(f32(self.dist_izq))
        self.pub_dr.publish(f32(self.dist_der))
        self.pub_wf.publish(f32(wf))
        self.pub_wd.publish(f32(wd))
        self.pub_wi.publish(f32(wi))
        self.pub_wt.publish(f32(wt))

    def _t_estado(self) -> float:
        return (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9

    def _w_der_pd(self) -> float:
        """PD de tracking a la pared derecha. El termino derivativo amortigua
        el overshoot que trae subir Kder para pegarse mas rapido al target."""
        error = self.dist_der - self.target_der
        now   = self.get_clock().now()
        dt    = max((now - self._t_der_prev).nanoseconds * 1e-9, 0.01)
        d_err = (error - self._err_der_prev) / dt
        self._err_der_prev = error
        self._t_der_prev   = now
        return -(self.Kder * error + self.Kd_der * d_err)

    def _cambiar(self, nuevo: str):
        self.get_logger().info(
            f'{self.estado}→{nuevo}  '
            f'f={self.dist_frente:.2f}  l={self.dist_izq:.2f}  r={self.dist_der:.2f}')
        # Marca cooldown al salir de GIRO o RODEO hacia CRUCERO
        if self.estado in (GIRO, RODEO) and nuevo == CRUCERO:
            self.t_ultimo_giro = self.get_clock().now().nanoseconds * 1e-9
            # Resetea la derivada: si no, el primer tick calcula d_err sobre
            # un salto acumulado durante todo GIRO+RODEO (spike falso).
            if math.isfinite(self.dist_der):
                self._err_der_prev = self.dist_der - self.target_der
            self._t_der_prev = self.get_clock().now()
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()

    def loop_control(self):

        # ── PRIORIDAD 1: STOP absoluto ────────────────────────────────────
        if self.dist_frente < self.d_stop_front:
            self._pub(0.0, 0.0)
            self._pub_dbg(0, 0, 0, 0)
            self.get_logger().warn(
                f'PARA frente={self.dist_frente:.3f}m', throttle_duration_sec=0.4)
            return
        # Usa el minimo crudo (dist_izq_raw/dist_der_raw), NO el de RANSAC:
        # RANSAC descarta a proposito objetos que no son pared (p.ej. una
        # caja pegada al costado) como outliers, y ese es justo el caso que
        # este STOP tiene que detectar.
        if math.isfinite(self.dist_izq_raw) and self.dist_izq_raw < self.d_stop_lat:
            self._pub(0.0, 0.0)
            self._pub_dbg(0, 0, 0, 0)
            self.get_logger().warn(
                f'PARA izq={self.dist_izq_raw:.3f}m', throttle_duration_sec=0.4)
            return
        if math.isfinite(self.dist_der_raw) and self.dist_der_raw < self.d_stop_lat:
            # Ahora el GIRO puede ir hacia cualquier lado (no solo izquierda),
            # asi que el riesgo de colision lateral tambien puede venir del
            # lado derecho.
            self._pub(0.0, 0.0)
            self._pub_dbg(0, 0, 0, 0)
            self.get_logger().warn(
                f'PARA der={self.dist_der_raw:.3f}m', throttle_duration_sec=0.4)
            return

        # ── CRUCERO ───────────────────────────────────────────────────────
        if self.estado == CRUCERO:
            ahora = self.get_clock().now().nanoseconds * 1e-9
            t_post = ahora - self.t_ultimo_giro
            cooldown_ok = t_post >= self.t_cooldown

            if cooldown_ok and self.dist_frente <= self.d_giro:
                # Girar hacia el lado con MAS espacio disponible en este
                # instante (no siempre izquierda). Sin referencia de ningun
                # lado, izquierda por defecto.
                if math.isfinite(self.dist_izq) and math.isfinite(self.dist_der):
                    self.dir_giro = 1.0 if self.dist_izq >= self.dist_der else -1.0
                elif math.isfinite(self.dist_der):
                    self.dir_giro = -1.0
                else:
                    self.dir_giro = 1.0
                # Magnitud del giro: se calcula UNA vez aqui, con las
                # lecturas de este instante, y queda fija durante todo el
                # GIRO. Recalcularla en cada ciclo con dist_frente en vivo
                # (como antes) la hacia erratica: dist_frente cambia rapido
                # y de forma poco representativa mientras el robot rota,
                # asi que el giro salia a veces corto, a veces excesivo.
                urgencia = max(0.0, self.d_giro - self.dist_frente) / max(self.d_giro, 1e-6)
                self.w_giro_efectivo = max(-self.max_w, min(self.max_w,
                    self.dir_giro * self.w_giro * (1.0 + self.k_urgencia_giro * urgencia)))
                self._cambiar(GIRO)
                self.get_logger().info(
                    f'GIRO dir={"izq" if self.dir_giro > 0 else "der"}'
                    f'  izq={self.dist_izq:.2f}  der={self.dist_der:.2f}'
                    f'  w_giro={self.w_giro_efectivo:.2f}')
                self._pub_dbg(0, 0, 0, 0)
                return

            recuperando = t_post < self.t_recuperacion
            w_front = 0.0
            w_der   = 0.0

            if recuperando:
                # Solo tracking pared derecha — permite alinearse tras RODEO
                if math.isfinite(self.dist_der):
                    w_der = self._w_der_pd()
            elif self.dist_frente < self.d_front_ini:
                # Evasion frontal pura — sin competencia con w_der
                w_front = self.Kfront * (self.d_front_ini - self.dist_frente)
            else:
                # Tracking normal pared derecha
                if math.isfinite(self.dist_der):
                    w_der = self._w_der_pd()

            # Repulsion izquierda (siempre activa)
            w_izq = 0.0
            if math.isfinite(self.dist_izq) and self.dist_izq < self.d_izq_min:
                w_izq = -self.Kizq * (self.d_izq_min - self.dist_izq)

            w = max(-self.max_w, min(self.max_w, w_front + w_der + w_izq))
            self._pub_dbg(w_front, w_der, w_izq, w)
            self._pub(self.v_cruise, w)

        # ── GIRO ──────────────────────────────────────────────────────────
        elif self.estado == GIRO:
            # d_lado: distancia del lado HACIA EL QUE SE GIRA (criterio de
            # salida por acercamiento excesivo a esa pared).
            d_lado = self.dist_izq if self.dir_giro > 0 else self.dist_der

            if self._t_estado() > self.t_giro_max:
                self._cambiar(RODEO)
                return
            if math.isfinite(d_lado) and d_lado < self.d_lado_salida_giro:
                self.get_logger().warn(
                    f'GIRO→RODEO lado={"izq" if self.dir_giro > 0 else "der"}={d_lado:.2f}m')
                self._cambiar(RODEO)
                return
            if self._t_estado() >= self.t_giro_min and self.dist_frente > self.d_giro:
                self._cambiar(RODEO)
                return

            # Magnitud congelada al entrar a GIRO (ver comentario en CRUCERO).
            # v_maniobra (mas lenta que crucero): menos distancia recorrida
            # "a ciegas" por ciclo si el rumbo queda torcido.
            self._pub_dbg(self.w_giro_efectivo, 0, 0, self.w_giro_efectivo)
            self._pub(self.v_maniobra, self.w_giro_efectivo)

        # ── RODEO: avance recto para separarse del obstaculo ──────────────
        elif self.estado == RODEO:
            t = self._t_estado()
            if t >= self.t_rodeo_max:
                self._cambiar(CRUCERO)
                return
            # Salida por sensor: recien evaluamos si el frente ya despejo
            # despues de un minimo, para no cortar el rodeo por un rebote
            # transitorio justo al salir de GIRO.
            if t >= self.t_rodeo_min and self.dist_frente > self.d_front_ini:
                self._cambiar(CRUCERO)
                return
            self._pub_dbg(0, 0, 0, 0)
            self._pub(self.v_maniobra, 0.0)   # w=0: absolutamente recto, mas lento que crucero


def _esperar_inicio(logger, segundos: float):
    """Cuenta regresiva antes de arrancar (tiempo para acomodar el robot en
    la pista). El nodo ya esta creado (topicos advertidos) pero no se
    procesa ningun callback hasta rclpy.spin(), asi que nada se mueve
    mientras tanto. Sin ENTER: funciona igual con ros2 run o ros2 launch."""
    restante = segundos
    while restante > 0:
        logger.info(f'Arrancando en {restante:.0f}s...')
        time.sleep(1.0)
        restante -= 1.0
    logger.info('Arrancando!')


def main(args=None):
    rclpy.init(args=args)
    nodo = BehaviorFSM()
    _esperar_inicio(nodo.get_logger(), nodo.t_espera_inicio)
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo._pub(0.0, 0.0)
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
