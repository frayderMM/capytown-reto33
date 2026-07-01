#!/usr/bin/env python3
"""
monitor.py — Monitor terminal en tiempo real para CapyTown Guardian v2.

Uso (en el robot, con ROS2 sourced):
  python3 monitor.py

Muestra distancias, componentes de correccion angular, estado FSM y cmd_vel.
Presiona Ctrl+C para salir.

Para ajustar parametros en caliente (en otra terminal):
  ros2 param set /behavior_fsm Kder 5.0
  ros2 param set /behavior_fsm target_der 0.10
  ros2 param set /behavior_fsm Kfront 1.5
  ros2 param set /behavior_fsm d_izq_min 0.12
  ros2 param set /behavior_fsm vel_crucero 0.12
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

INF = float('inf')
BW  = 25  # ancho de barras

# Umbrales fijos para colorear (deben coincidir con params.yaml)
THRES = {
    'frente': {'stop': 0.08, 'giro': 0.22, 'grad': 0.40},
    'izq':    {'stop': 0.05, 'rep':  0.15},
    'der':    {'lo':   0.10, 'target': 0.13, 'hi': 0.16},
}

R  = '\033[91m'   # rojo
Y  = '\033[93m'   # amarillo
G  = '\033[92m'   # verde
B  = '\033[94m'   # azul
M  = '\033[95m'   # magenta
C  = '\033[96m'   # cyan
W  = '\033[97m'   # blanco brillante
DIM= '\033[2m'    # tenue
X  = '\033[0m'    # reset


def bar(val, mn, mx, width=BW, lo_warn=None, hi_warn=None, target=None):
    """Barra proporcional con marca de target."""
    if not math.isfinite(val):
        return DIM + '[' + '?' * width + ']' + X
    frac  = max(0.0, min(1.0, (val - mn) / (mx - mn + 1e-9)))
    pos   = int(frac * width)
    tpos  = int(((target - mn) / (mx - mn + 1e-9)) * width) if target else -1

    chars = []
    for i in range(width):
        if i == tpos:
            chars.append('|')
        elif i < pos:
            chars.append('█')
        else:
            chars.append('░')
    # Color de la barra segun valor
    if lo_warn and val < lo_warn:
        col = R
    elif hi_warn and val > hi_warn:
        col = Y
    else:
        col = G
    return col + '[' + ''.join(chars) + ']' + X


def cfmt(val, mn, mx, unit='m', precision=3):
    """Valor coloreado segun rangos."""
    if not math.isfinite(val) or val < -900:
        return DIM + '  ---  ' + X
    s = f'{val:.{precision}f}{unit}'
    if val <= mn:
        return R + s + X
    if val <= mn * 1.5:
        return Y + s + X
    if val >= mx:
        return Y + s + X
    return G + s + X


def wcol(val):
    """Color para correccion angular."""
    if abs(val) > 0.4:
        return Y
    if abs(val) > 0.15:
        return C
    return DIM


class Monitor(Node):
    def __init__(self):
        super().__init__('capytown_monitor')
        self.d_f     = INF; self.d_l = INF; self.d_r = INF
        self.w_front = 0.0; self.w_der = 0.0
        self.w_izq   = 0.0; self.w_total = 0.0
        self.vel_lin = 0.0; self.vel_ang = 0.0
        self.estado  = '---'
        self._tick   = 0
        self._t0     = time.time()

        def mk(attr):
            def cb(msg):
                v = msg.data
                setattr(self, attr, v if v > -900 else INF)
            return cb

        q = 10
        self.create_subscription(Float32, '/dist_frente',  mk('d_f'),     q)
        self.create_subscription(Float32, '/dist_izq',     mk('d_l'),     q)
        self.create_subscription(Float32, '/dist_der',     mk('d_r'),     q)
        self.create_subscription(Float32, '/dbg/w_front',  mk('w_front'), q)
        self.create_subscription(Float32, '/dbg/w_der',    mk('w_der'),   q)
        self.create_subscription(Float32, '/dbg/w_izq',    mk('w_izq'),   q)
        self.create_subscription(Float32, '/dbg/w_total',  mk('w_total'), q)
        self.create_subscription(Twist,  '/cmd_vel',       self._cmd,     q)
        self.create_subscription(String, '/fsm_state',     self._est,     q)
        self.create_timer(0.15, self._draw)

    def _cmd(self, msg):
        self.vel_lin = msg.linear.x
        self.vel_ang = msg.angular.z

    def _est(self, msg):
        self.estado = msg.data

    def _draw(self):
        self._tick += 1
        os.system('cls' if os.name == 'nt' else 'clear')
        t  = time.time() - self._t0
        up = f'{int(t//60):02d}:{int(t%60):02d}'

        # Estado FSM
        if self.estado == 'GIRO':
            est = Y + f'● GIRO   ' + X
        elif self.estado == 'CRUCERO':
            est = G + f'● CRUCERO' + X
        else:
            est = M + f'● {self.estado:<7}' + X

        # Distancias
        df = self.d_f; dl = self.d_l; dr = self.d_r

        df_bar = bar(df, 0, 0.6, lo_warn=THRES['frente']['stop'],
                     hi_warn=THRES['frente']['grad'])
        dl_bar = bar(dl, 0, 0.6, lo_warn=THRES['izq']['stop'],
                     hi_warn=THRES['izq']['rep'])
        dr_bar = bar(dr, 0, 0.3, lo_warn=THRES['der']['lo'],
                     hi_warn=THRES['der']['hi'],
                     target=THRES['der']['target'])

        # Valores
        def fmtd(v):
            if not math.isfinite(v): return DIM + '  --- ' + X
            cm = v * 100
            if cm < 8:   return R + f'{cm:5.1f}cm' + X
            if cm < 22:  return Y + f'{cm:5.1f}cm' + X
            return G + f'{cm:5.1f}cm' + X
        def fmtl(v):
            if not math.isfinite(v): return DIM + '  --- ' + X
            cm = v * 100
            if cm < 5:   return R + f'{cm:5.1f}cm' + X
            if cm < 15:  return Y + f'{cm:5.1f}cm' + X
            return G + f'{cm:5.1f}cm' + X
        def fmtr(v):
            if not math.isfinite(v): return DIM + '  --- ' + X
            cm = v * 100
            tgt = THRES['der']['target'] * 100
            err = cm - tgt
            if abs(err) <= 2: return G + f'{cm:5.1f}cm' + X
            if abs(err) <= 5: return Y + f'{cm:5.1f}cm' + X
            return R + f'{cm:5.1f}cm' + X

        # Correcciones
        def fmtw(v, label):
            c = wcol(v)
            arrow = '←' if v > 0.01 else ('→' if v < -0.01 else ' ')
            return f'{c}{label}{v:+.3f} {arrow}{X}'

        print(f'{W}╔══════════════════════════════════════════════════════╗{X}')
        print(f'{W}║{X}  CAPYTOWN MONITOR  estado={est}  up={up}  #{self._tick:<5} {W}║{X}')
        print(f'{W}╠══════════════════════════════════════════════════════╣{X}')
        print(f'{W}║{X}  DISTANCIAS                                         {W}║{X}')
        print(f'{W}║{X}  FRENTE  {fmtd(df)}  {df_bar}  {W}║{X}')
        print(f'{W}║{X}  {DIM}         stop<8cm  giro<22cm  grad<40cm{X}         {W}║{X}')
        print(f'{W}║{X}  IZQ     {fmtl(dl)}  {dl_bar}  {W}║{X}')
        print(f'{W}║{X}  {DIM}         stop<5cm  repuls<15cm{X}                  {W}║{X}')
        print(f'{W}║{X}  DER     {fmtr(dr)}  {dr_bar}  {W}║{X}')
        print(f'{W}║{X}  {DIM}         target=13cm  OK:10-16cm  |=target{X}       {W}║{X}')
        print(f'{W}╠══════════════════════════════════════════════════════╣{X}')
        print(f'{W}║{X}  CORRECCIONES ANGULARES  {DIM}(+izq  -der){X}             {W}║{X}')
        print(f'{W}║{X}  {fmtw(self.w_front, "w_front  ")}  evasion frontal       {W}║{X}')
        print(f'{W}║{X}  {fmtw(self.w_der,   "w_der    ")}  tracking pared der    {W}║{X}')
        print(f'{W}║{X}  {fmtw(self.w_izq,   "w_izq    ")}  repulsion izq         {W}║{X}')
        print(f'{W}║{X}  ─────────────────────────                        {W}║{X}')
        print(f'{W}║{X}  {fmtw(self.w_total, "w_TOTAL  ")}  cmd angular           {W}║{X}')
        print(f'{W}╠══════════════════════════════════════════════════════╣{X}')
        vl = self.vel_lin; va = self.vel_ang
        vc = G if abs(vl) > 0.01 else DIM
        ac = Y if abs(va) > 0.05 else DIM
        print(f'{W}║{X}  CMD_VEL  {vc}linear={vl:+.3f}m/s{X}  {ac}angular={va:+.3f}rad/s{X}     {W}║{X}')
        print(f'{W}╠══════════════════════════════════════════════════════╣{X}')
        print(f'{W}║{X}  AJUSTE EN CALIENTE (otra terminal):                {W}║{X}')
        print(f'{DIM}  ros2 param set /behavior_fsm Kder 5.0             {X}')
        print(f'{DIM}  ros2 param set /behavior_fsm target_der 0.10      {X}')
        print(f'{DIM}  ros2 param set /behavior_fsm Kfront 1.5           {X}')
        print(f'{DIM}  ros2 param set /behavior_fsm d_izq_min 0.12       {X}')
        print(f'{W}╚══════════════════════════════════════════════════════╝{X}')
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
