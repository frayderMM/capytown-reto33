#!/usr/bin/env python3
"""
test_giros.py — Prueba de maniobras de rodeo para caja 20x20 cm.

Cada maniobra:
  1. Cuenta regresiva 5s  (coloca el robot a ~30cm de la caja, mirando al frente)
  2. Ejecuta la secuencia de fases
  3. Para el robot
  4. Espera ENTER para la siguiente

Uso:
  python3 test_giros.py           # todas en orden
  python3 test_giros.py 3         # solo maniobra 3

Geometria de referencia:
  - Caja: 20x20 cm
  - Distancia inicial al frente: ~30 cm
  - Objetivo: quedar paralelo a la pared derecha despues de la maniobra

Angulo = w (rad/s) * t (s) en radianes. Conversion: 1 rad = 57.3 grados.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ─────────────────────────────────────────────────────────────────────────────
# MANIOBRAS  —  cada fase: (v m/s, w rad/s, duracion s)
# ─────────────────────────────────────────────────────────────────────────────
# Notacion angular: w>0 = gira izquierda,  w<0 = gira derecha
# Angulo total de giro: w * t  (rad)
#
# Caja 20x20 cm:
#   Con 30 cm de distancia al frente, el robot debe desplazarse
#   lateralmente ~20cm + margen para dejar libre la caja.
#   Desplazamiento lateral de un arco = v/w * (1-cos(theta))

MANIOBRAS = [
    {
        'id': 1,
        'nombre': 'Suave  20°  (w=0.40 rad/s)',
        'notas': 'Giro suave, poca separacion lateral. Para corredor ancho.',
        'fases': [
            # fase              v      w      t(s)    angulo
            ('gira_izq',     0.10,  +0.40,  0.87),  # +0.35 rad = 20°
            ('recto',        0.10,   0.00,  2.50),  # 25 cm adelante
            ('gira_der',     0.10,  -0.40,  0.87),  # -0.35 rad = 20° (vuelve paralelo)
            ('acerca_pared', 0.10,  -0.15,  1.50),  # suave hacia pared der
            ('stop',         0.00,   0.00,  0.50),
        ]
    },
    {
        'id': 2,
        'nombre': 'Estandar  30°  (w=0.50 rad/s)',
        'notas': 'Balance entre separacion y angulo. Punto de partida tipico.',
        'fases': [
            ('gira_izq',     0.10,  +0.50,  1.05),  # +0.52 rad = 30°
            ('recto',        0.10,   0.00,  2.50),  # 25 cm
            ('gira_der',     0.10,  -0.50,  1.05),  # -0.52 rad = 30°
            ('acerca_pared', 0.10,  -0.15,  1.00),
            ('stop',         0.00,   0.00,  0.50),
        ]
    },
    {
        'id': 3,
        'nombre': 'Amplio  45°  (w=0.50 rad/s)',
        'notas': 'Mayor separacion lateral. Para cajas bien centradas.',
        'fases': [
            ('gira_izq',     0.10,  +0.50,  1.57),  # +0.79 rad = 45°
            ('recto',        0.10,   0.00,  3.00),  # 30 cm
            ('gira_der',     0.10,  -0.50,  1.57),  # -0.79 rad = 45°
            ('acerca_pared', 0.10,  -0.10,  1.50),
            ('stop',         0.00,   0.00,  0.50),
        ]
    },
    {
        'id': 4,
        'nombre': 'Arco suave  25°  (w=0.30 rad/s)',
        'notas': 'Arco muy amplio, radio 33cm. Mas estable en linea recta.',
        'fases': [
            ('gira_izq',     0.10,  +0.30,  1.45),  # +0.44 rad = 25°
            ('recto',        0.10,   0.00,  2.50),  # 25 cm
            ('gira_der',     0.10,  -0.30,  1.45),  # -0.44 rad = 25°
            ('recto',        0.10,   0.00,  1.00),
            ('stop',         0.00,   0.00,  0.50),
        ]
    },
    {
        'id': 5,
        'nombre': 'Rapido  35°  (w=0.60 rad/s)',
        'notas': 'Giro rapido y recto largo. Para pasillos mas estrechos.',
        'fases': [
            ('gira_izq',     0.10,  +0.60,  1.02),  # +0.61 rad = 35°
            ('recto',        0.10,   0.00,  2.50),  # 25 cm
            ('gira_der',     0.10,  -0.60,  1.02),  # -0.61 rad = 35°
            ('acerca_pared', 0.10,  -0.20,  1.00),
            ('stop',         0.00,   0.00,  0.50),
        ]
    },
]


# ─────────────────────────────────────────────────────────────────────────────
# NODO
# ─────────────────────────────────────────────────────────────────────────────
class TestGiros(Node):
    def __init__(self):
        super().__init__('test_giros')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def stop(self):
        self.cmd(0.0, 0.0)

    def ejecutar_fase(self, nombre_fase: str, v: float, w: float, duracion: float):
        print(f'    [{nombre_fase}]  v={v:+.2f}  w={w:+.2f}  t={duracion:.2f}s')
        t_ini = time.time()
        rate  = 0.05  # 20 Hz
        while time.time() - t_ini < duracion:
            self.cmd(v, w)
            time.sleep(rate)
        self.stop()

    def ejecutar_maniobra(self, m: dict):
        print(f'\n{"="*55}')
        print(f'  MANIOBRA {m["id"]}: {m["nombre"]}')
        print(f'  {m["notas"]}')
        print(f'{"="*55}')
        print('  Coloca el robot a ~30 cm de la caja, mirando al frente.')
        print('  Iniciando en:', end='', flush=True)
        for i in range(5, 0, -1):
            print(f' {i}...', end='', flush=True)
            time.sleep(1.0)
        print(' ¡YA!\n')

        for fase in m['fases']:
            nombre_fase, v, w, t = fase
            self.ejecutar_fase(nombre_fase, v, w, t)
            time.sleep(0.1)

        self.stop()
        print(f'\n  Maniobra {m["id"]} completada.')
        print('  Observa si el robot quedo paralelo a la pared derecha.')


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = TestGiros()

    # Elegir maniobras a ejecutar
    if len(sys.argv) > 1:
        ids = [int(x) for x in sys.argv[1:]]
        maniobras = [m for m in MANIOBRAS if m['id'] in ids]
    else:
        maniobras = MANIOBRAS

    print('\n' + '='*55)
    print('  TEST DE GIROS — caja 20x20 cm')
    print(f'  {len(maniobras)} maniobras a ejecutar')
    print('='*55)
    print('  Resumen:')
    for m in maniobras:
        print(f'    {m["id"]}. {m["nombre"]}')
    print()

    try:
        for i, m in enumerate(maniobras):
            node.ejecutar_maniobra(m)

            if i < len(maniobras) - 1:
                input('\n  Presiona ENTER cuando el robot este listo para la siguiente...')

    except KeyboardInterrupt:
        print('\n  Interrumpido.')
    finally:
        node.stop()
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()

    print('\n  Fin del test. Dime cual maniobra funciono mejor.')


if __name__ == '__main__':
    main()
