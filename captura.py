#!/usr/bin/env python3
"""
captura.py — Imprime datos en texto plano para copiar y pegar.

Uso:
  python3 captura.py          # captura 60 lineas (~12 segundos) y para
  python3 captura.py 100      # captura N lineas
  python3 captura.py 0        # sin limite (Ctrl+C para parar)

Luego copia la salida completa y pegala en el chat.
"""

import math, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

MAX = int(sys.argv[1]) if len(sys.argv) > 1 else 60
INF = float('inf')

class Captura(Node):
    def __init__(self):
        super().__init__('captura')
        self.d_f = INF; self.d_l = INF; self.d_r = INF
        self.w_front = 0.0; self.w_der = 0.0; self.w_izq = 0.0; self.w_total = 0.0
        self.vel_lin = 0.0; self.vel_ang = 0.0
        self.estado = '---'
        self.n = 0
        self._t0 = time.time()

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
        self.create_timer(0.2, self._log)

        # Cabecera
        print('# t(s)  estado    f(cm)  l(cm)  r(cm) | wfr   wder   wizq   wtot | vlin  vang')
        print('# -----------------------------------------------------------------------')

    def _cmd(self, msg):
        self.vel_lin = msg.linear.x; self.vel_ang = msg.angular.z

    def _est(self, msg):
        self.estado = msg.data

    def _fmt(self, v):
        return f'{v*100:6.1f}' if math.isfinite(v) else '  ---'

    def _log(self):
        t = time.time() - self._t0
        f  = self._fmt(self.d_f)
        l  = self._fmt(self.d_l)
        r  = self._fmt(self.d_r)
        print(f'{t:6.1f}  {self.estado:<8}  {f}  {l}  {r} |'
              f' {self.w_front:+.3f} {self.w_der:+.3f} {self.w_izq:+.3f} {self.w_total:+.3f} |'
              f' {self.vel_lin:+.3f} {self.vel_ang:+.3f}')
        sys.stdout.flush()
        self.n += 1
        if MAX > 0 and self.n >= MAX:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = Captura()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

if __name__ == '__main__':
    main()
