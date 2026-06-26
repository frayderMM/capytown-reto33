# CapyTown RC3 — Hoja de Ruta

---

## Estado actual del proyecto

| Componente | Estado |
|---|---|
| `box_detector` — censo de cajas con LiDAR | Listo (pendiente prueba en robot) |
| `behavior_fsm` — FSM 3 estados + vel. adaptativa | Listo (pendiente prueba en robot) |
| `wall_follower` — centrado Split-and-Merge + PD | Listo (pendiente prueba en robot) |
| `metrics_logger` — CSV con VP/FP/FN | Listo (ground truth a configurar) |
| `lidar_viz` — monitor visual tiempo real | Listo |
| Orientación del LiDAR | Parámetro `lidar_front_deg` implementado (confirmar valor en robot) |
| Código en el robot | **SIN SUBIR** |

---

## Bloqueantes críticos 🔴
> Sin resolver estos dos, nada funciona en el robot.

### 1. Subir el código al robot (git push)
Todos los cambios están solo en el PC. Hay que empujar antes de cualquier prueba.

```bash
# En el PC
git add .
git commit -m "refactor: reestructura a dos paquetes, FSM mejorada, lidar_viz"
git push origin main

# En el robot (dentro del docker friendly_pike)
cd /root/frayder_ws/src/RC3
git fetch origin && git reset --hard origin/main
cd /root/frayder_ws
colcon build --packages-select box_detector behavior_fsm
source install/setup.bash
```

---

### 2. Confirmar orientación del LiDAR
El código asume que el frente del robot está en un ángulo del LiDAR.
El código antiguo usaba ±180°, el nuevo asume 0°. Sin confirmar cuál es correcto,
el robot reacciona en la dirección equivocada.

**Cómo verificar (en el robot con obstáculo enfrente):**
```bash
# Opción rápida: ver ángulo con distancia mínima
ros2 topic echo /scan | grep -A3 ranges

# O lanzar lidar_viz en el PC y ver dónde caen los puntos rojos (sector frente)
python3 lidar_viz.py           # prueba con frente=180°
python3 lidar_viz.py --front 0 # prueba con frente=0°
```

**Fix:** agregar parámetro `lidar_front_deg` en `params.yaml` y aplicarlo
en `behavior_fsm.py` y `wall_follower.py` para no hardcodear el ángulo.

---

## Tareas importantes 🟡
> Afectan directamente la nota del reto.

### 3. Configurar ground truth antes de cada corrida
El `metrics_logger` compara las detecciones contra posiciones reales de las cajas.
El archivo `box_detector/config/params.yaml` tiene valores ficticios por defecto:
```yaml
ground_truth: [1.0, 0.5, 2.0, -0.5, 3.0, 0.4]  # ← cambiar antes de cada corrida
```
Antes de cada corrida: medir con cinta métrica las posiciones (x, y) en odom
de las 5 cajas y actualizar este parámetro.

---

### 4. Reiniciar box_detector entre corridas
El censo acumula cajas **en memoria** mientras el nodo vive.
Si haces la corrida 2 sin reiniciar, el censo ya tiene las cajas de la corrida 1.

```bash
# Entre corridas: matar y relanzar el nodo
ros2 lifecycle set /box_detector shutdown   # si tiene lifecycle
# O simplemente Ctrl+C y volver a lanzar:
ros2 launch behavior_fsm capytown.launch.py
```

---

### 5. Evaluar si dist_parada necesita más margen
`dist_parada: 0.18m` da solo 3cm de margen sobre el requisito de ≥15cm.
Con latencia del LiDAR y vibración del robot, puede quedarse corto.
Considerar subir a `0.22m` para la primera prueba y ajustar.

---

## Secuencia de pruebas en el robot 🧪

### Prueba 1 — Solo LiDAR (sin mover el robot)
```bash
ros2 launch box_detector lidar.launch.py
```
Verificar en los logs que detecta cajas y el censo cuenta bien.
Abrir `lidar_viz.py` en el PC para ver los puntos en tiempo real.

---

### Prueba 2 — Centrado entre paredes (wall_follower solo)
```bash
ros2 run behavior_fsm wall_follower \
  --ros-args --params-file \
  ~/frayder_ws/install/behavior_fsm/share/behavior_fsm/config/params.yaml
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Mover el robot manualmente con el teclado y verificar que `/lateral_correction`
se activa y centra el robot entre las paredes.

---

### Prueba 3 — FSM frente a una sola caja
```bash
ros2 launch behavior_fsm capytown.launch.py
```
Poner una caja a ~50cm enfrente. Verificar:
- [ ] Frena progresivamente (velocidad adaptativa)
- [ ] Se detiene a ≥15cm
- [ ] Mide qué lado tiene más espacio (log: `izq=X.XXm der=X.XXm`)
- [ ] Gira hacia el lado correcto
- [ ] Bordea la caja y se reincorpora al carril

---

### Prueba 4 — Ajuste de parámetros de rodeo
Según lo observado en Prueba 3, ajustar en `behavior_fsm/config/params.yaml`:

| Parámetro | Valor actual | Ajustar si... |
|---|---|---|
| `angulo_rodeo_deg` | 35° | Robot no esquiva con suficiente margen → subir |
| `avance_rodeo_seg` | 1.8s | Robot se reincorpora antes de pasar la caja → subir |
| `vel_crucero` | 0.18 m/s | Va muy rápido para el jirón → bajar a 0.15 |
| `dist_parada` | 0.18m | Para muy cerca → subir a 0.22m |
| `Kp` wall_follower | 0.80 | Oscila mucho al centrar → bajar |

---

### Prueba 5 — Corrida completa (10 corridas para entregable)
```bash
ros2 launch behavior_fsm capytown.launch.py
```
- Actualizar `ground_truth` en params.yaml con posiciones reales
- Realizar corrida
- Copiar CSV: `scp root@10.42.0.1:/root/metricas_lidar.csv ./corrida_N.csv`
- Reiniciar nodos y repetir

---

## Entregables pendientes 📦

| Entregable | Estado |
|---|---|
| Paquete `box_detector` documentado | Listo |
| Paquete `behavior_fsm` (FSM mejorada) | Listo |
| `metricas_lidar.csv` con 10 corridas | Pendiente (necesita robot) |
| Captura de RViz con markers de cajas | Pendiente (necesita robot) |
| Video compilado (5 cajas, sin tumbar ninguna) | Pendiente (necesita robot) |
| Bonus elegido (sigue-corredor o estacionamiento) | Sin definir |

---

## Comandos importantes

### Conexión al robot
```bash
ssh root@10.42.0.1
```

### Docker (en el robot)
```bash
docker start friendly_pike        # iniciar
docker exec -it friendly_pike bash  # entrar
docker stop friendly_pike         # detener
```

### ROS2 en el robot
```bash
source /opt/ros/humble/setup.bash
source /root/frayder_ws/install/setup.bash

# Actualizar desde git y compilar
cd /root/frayder_ws/src/RC3
git fetch origin && git reset --hard origin/main
cd /root/frayder_ws
colcon build --packages-select box_detector behavior_fsm
source install/setup.bash
```

### Lanzar el proyecto
```bash
# Solo detector de cajas (Parte A)
ros2 launch box_detector lidar.launch.py

# Todo junto (corrida completa)
ros2 launch behavior_fsm capytown.launch.py
```

### Monitoreo de topics
```bash
ros2 topic list                                      # ver todos los topics
ros2 topic echo /cajas_avistadas                     # cajas detectadas
ros2 topic echo /cmd_vel                             # velocidades al robot
ros2 topic echo /lateral_correction                  # corrección wall_follower
ros2 topic echo /fsm_state                           # estado actual de la FSM
```

### Control manual con teclado
```bash
# Dentro del docker en el robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i=adelante  ,=atras  j=izq  l=der  k=frenar
```

### Parar el robot (emergencia)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

### Visualización LiDAR en tiempo real (desde el PC)
```bash
# Requiere: pip install matplotlib numpy
python3 lidar_viz.py            # frente en ±180° (Yahboom por defecto)
python3 lidar_viz.py --front 0  # frente en 0°
```

### Copiar archivos del robot al PC
```bash
scp root@10.42.0.1:/root/metricas_lidar.csv .
scp root@10.42.0.1:/root/frayder_ws/src/RC3/*.png .
```
