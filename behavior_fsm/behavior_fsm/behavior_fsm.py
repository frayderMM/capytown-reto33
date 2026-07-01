#!/usr/bin/env python3
"""
behavior_fsm.py  —  PARTE B: "El Guardian"

Vector Field Histogram (VFH) simplificado — Borenstein & Koren, 1991.
Algoritmo estandar de evasion reactiva local para robots con escaner de
rango 2D, adaptado aqui como "single-scan VFH": el histograma polar se
reconstruye de cada LaserScan (sin grid cartesiano persistente ni
decaimiento temporal, que en el paper original existian para modelar el
ruido angular ancho de sonares — con un LiDAR 2D preciso no aportan
nada). Se usa el minimo de rango por sector como "ocupacion" en vez de
la funcion de densidad ponderada del paper original.

Pipeline por ciclo:
  1. Histograma polar: 72 sectores de 5°, minimo de rango por sector.
  2. Binario: sector "bloqueado" si su espacio libre real (post-offset
     LiDAR->borde del robot) es menor a dist_obstaculo.
  3. Enmascarado: un sector solo es hueco valido si el y sus vecinos,
     hasta cubrir el ancho fisico del robot + margen, estan libres --
     no basta con que un rayo lea "lejos".
  4. Candidato: entre los huecos validos, el mas cercano a 0° (adelante;
     no hay meta de navegacion, solo avanzar).
  5. Salida continua: v y w como funcion suave del angulo/distancia del
     candidato elegido -- sin modos discretos, sin saltos bruscos.

Sin huecos validos (o el elegido sigue dentro de dist_emergencia): parada
total, no hay forma segura de girar para evitarlo.

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('lidar_front_deg',    180.0)
        self.declare_parameter('sector_frontal_deg',  30.0)  # +/- grados: define
                                                               # las zonas frente/atras/lados
                                                               # para elegir el offset de cada sector
        self.declare_parameter('excluir_atras_deg',   60.0)  # +/- grados detras del robot
                                                               # excluidos del histograma: el
                                                               # cable/soporte del LiDAR cae ahi
                                                               # y se leia como obstaculo fijo

        # --- Offsets LiDAR -> borde fisico del robot (NO es el mismo en
        # cada direccion) -- las distancias del LiDAR se corrigen con esto
        # antes de comparar contra cualquier umbral.
        self.declare_parameter('offset_frente', 0.15)  # m  LiDAR -> borde frontal
        self.declare_parameter('offset_atras',  0.10)  # m  LiDAR -> borde trasero
        self.declare_parameter('offset_lados',  0.08)  # m  LiDAR -> borde lateral

        # --- Histograma polar (VFH) ---
        self.declare_parameter('alpha_hist_deg',      5.0)   # grados por sector (360/5 = 72)
        self.declare_parameter('ancho_robot_m',       0.16)  # m  ancho fisico total del chasis
                                                               # (asumido: 2 x offset_lados,
                                                               # LiDAR centrado lateralmente)
        self.declare_parameter('margen_paso_extra_m', 0.05)  # m  colchon extra a cada lado
                                                               # del ancho del robot al validar un hueco

        # --- Distancias de reaccion (en espacio libre real, post-offset) ---
        self.declare_parameter('dist_alerta',     0.38)  # m  empieza a frenar/girar progresivamente
        self.declare_parameter('dist_obstaculo',  0.30)  # m  umbral de sector "bloqueado" en el
                                                           # histograma binario Y de giro maximo
        self.declare_parameter('dist_emergencia', 0.05)  # m  margen real minimo del candidato
                                                           # elegido antes del stop total

        # --- Velocidades ---
        self.declare_parameter('vel_crucero', 0.22)
        self.declare_parameter('vel_min',     0.08)
        self.declare_parameter('w_giro_max',  0.45)  # rad/s  saturacion del giro
        self.declare_parameter('Kp_giro',     0.45 / (math.pi / 2))  # rad/s por rad de error angular

        self.front_rad = math.radians(self.get_parameter('lidar_front_deg').value)
        self.sector_frontal = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.atras_lim = math.pi - math.radians(self.get_parameter('excluir_atras_deg').value) / 2.0

        self.off_frente = self.get_parameter('offset_frente').value
        self.off_atras  = self.get_parameter('offset_atras').value
        self.off_lados  = self.get_parameter('offset_lados').value

        self.alpha = math.radians(self.get_parameter('alpha_hist_deg').value)
        self.n_sect = int(round(2 * math.pi / self.alpha))
        self.ancho_robot = self.get_parameter('ancho_robot_m').value
        self.margen_paso = self.get_parameter('margen_paso_extra_m').value

        self.d_alerta = self.get_parameter('dist_alerta').value
        self.d_obst   = self.get_parameter('dist_obstaculo').value
        self.d_emerg  = self.get_parameter('dist_emergencia').value

        self.v_cruise  = self.get_parameter('vel_crucero').value
        self.v_min     = self.get_parameter('vel_min').value
        self.w_giro_max = self.get_parameter('w_giro_max').value
        self.Kp_giro    = self.get_parameter('Kp_giro').value

        # Geometria fija por sector (angulo central + offset segun zona
        # frente/atras/lados) -- se calcula una sola vez, no por scan.
        self.theta_sector  = [0.0] * self.n_sect
        self.offset_sector = [0.0] * self.n_sect
        for k in range(self.n_sect):
            theta = math.atan2(math.sin(k * self.alpha), math.cos(k * self.alpha))
            self.theta_sector[k] = theta
            abs_theta = abs(theta)
            if abs_theta <= self.sector_frontal:
                self.offset_sector[k] = self.off_frente
            elif abs_theta >= (math.pi - self.sector_frontal):
                self.offset_sector[k] = self.off_atras
            else:
                self.offset_sector[k] = self.off_lados

        # ── Sensores ──────────────────────────────────────────────────────
        self.hist_c = [float('inf')] * self.n_sect  # espacio libre real por sector

        # ── ROS I/O ───────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.cb_scan, _qos)
        self.pub_cmd    = self.create_publisher(Twist,   '/cmd_vel',     10)
        self.pub_estado = self.create_publisher(String,  '/fsm_state',   10)
        self.pub_parada = self.create_publisher(Float32, '/parada_dist', 10)
        self.create_timer(0.1, self.loop_control)

        self._en_evasion = False  # para publicar /parada_dist solo al entrar a evasion

        self.get_logger().info(
            f'BehaviorFSM listo — VFH simplificado ({self.n_sect} sectores, '
            f'ancho_robot={self.ancho_robot:.2f}m)')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def cb_scan(self, msg: LaserScan):
        hist_min = [float('inf')] * self.n_sect

        for i, r in enumerate(msg.ranges):
            raw    = msg.angle_min + i * msg.angle_increment
            af     = math.atan2(math.sin(raw - self.front_rad),
                                math.cos(raw - self.front_rad))
            valid  = math.isfinite(r) and msg.range_min <= r <= msg.range_max
            if not valid:
                continue

            # El cono trasero (cable/soporte del LiDAR) no aporta al
            # histograma -- esos sectores quedan en inf (libres/desconocidos).
            if abs(af) > self.atras_lim:
                continue

            k = int(round(af / self.alpha)) % self.n_sect
            if r < hist_min[k]:
                hist_min[k] = r

        self.hist_c = [
            (hist_min[k] - self.offset_sector[k]) if math.isfinite(hist_min[k]) else float('inf')
            for k in range(self.n_sect)
        ]

    # ── Helpers ───────────────────────────────────────────────────────────
    def _pub(self, v: float, w: float, estado: str):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String(); s.data = estado
        self.pub_estado.publish(s)

    def _vel_adaptativa(self, c: float) -> float:
        if c >= self.d_alerta:
            return self.v_cruise
        ratio = (c - self.d_obst) / (self.d_alerta - self.d_obst)
        return self.v_min + max(0.0, min(1.0, ratio)) * (self.v_cruise - self.v_min)

    def _elegir_candidato(self):
        """Recorre el histograma y devuelve (theta, c) del mejor hueco valido, o None."""
        mejor = None  # (abs(theta), -c, theta, c)  -- para minimizar abs(theta), luego maximizar c
        for k in range(self.n_sect):
            c_k = self.hist_c[k]
            if c_k < self.d_obst:
                continue  # sector bloqueado

            s_min = self.ancho_robot / 2.0 + self.margen_paso
            c_ref = c_k if math.isfinite(c_k) else 5.0  # techo razonable si no hay lectura
            medio_angulo = math.atan2(s_min, max(c_ref, 0.01))
            n_necesarios = int(math.ceil(medio_angulo / self.alpha)) * 2 + 1
            media_ventana = n_necesarios // 2

            valido = True
            for d in range(1, media_ventana + 1):
                if (self.hist_c[(k - d) % self.n_sect] < self.d_obst
                        or self.hist_c[(k + d) % self.n_sect] < self.d_obst):
                    valido = False
                    break
            if not valido:
                continue

            theta = self.theta_sector[k]
            clave = (abs(theta), -c_k)
            if mejor is None or clave < mejor[0]:
                mejor = (clave, theta, c_k)

        if mejor is None:
            return None
        return mejor[1], mejor[2]

    # ── Control principal ────────────────────────────────────────────────
    def loop_control(self):
        candidato = self._elegir_candidato()

        if candidato is None or candidato[1] < self.d_emerg:
            margen = candidato[1] if candidato is not None else float('nan')
            self.get_logger().warn(
                f'EMERGENCIA — sin hueco valido o margen={margen:.2f}m — stop total',
                throttle_duration_sec=1.0)
            self._pub(0.0, 0.0, 'EMERGENCIA')
            self._en_evasion = False
            return

        theta, c = candidato

        v = self._vel_adaptativa(c) * max(0.4, math.cos(theta))
        w = max(-self.w_giro_max, min(self.w_giro_max, self.Kp_giro * theta))

        if c < self.d_alerta:
            if not self._en_evasion:
                self._en_evasion = True
                d_msg = Float32(); d_msg.data = float(c)
                self.pub_parada.publish(d_msg)
        else:
            self._en_evasion = False

        self.get_logger().info(
            f'theta={math.degrees(theta):+.0f}°  c={c:.2f}  v={v:.2f} w={w:.2f}',
            throttle_duration_sec=0.5)
        self._pub(v, w, 'EVADIR' if self._en_evasion else 'AVANCE')


def main(args=None):
    rclpy.init(args=args)
    nodo = BehaviorFSM()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo._pub(0.0, 0.0, 'STOP')
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
