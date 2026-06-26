# CapyTown RC3 — Hoja de Ruta

---

## Estado actual del proyecto

| Componente | Estado |
|---|---|
| `box_detector` — censo de cajas con LiDAR | Listo |
| `behavior_fsm` — FSM 3 estados + vel. adaptativa | Listo |
| `wall_follower` — centrado Split-and-Merge + PD | Listo |
| `metrics_logger` — CSV con timestamp por corrida | Listo |
| `lidar_viz` — monitor matplotlib tiempo real | Listo |
| Código en el robot (git push) | **Subido** |
| Parámetro `lidar_front_deg` | Implementado (valor a confirmar en robot) |

---

## Incógnitas a resolver en el robot

Estas cinco cosas **no se pueden saber sin el robot** — resolverlas en orden al llegar:

| # | Incógnita | Cómo resolverla |
|---|---|---|
| 1 | Nombre del driver del LiDAR | `ros2 pkg list \| grep -i lidar` |
| 2 | Orientación del LiDAR (0° o 180°) | Obstáculo enfrente + ver `dist_frente` en logs FSM |
| 3 | `angulo_rodeo_deg` suficiente | Ver si bordea la caja sin chocarla |
| 4 | `avance_rodeo_seg` suficiente | Ver si se reincorpora antes de pasar la caja |
| 5 | `dist_parada` demasiado ajustado | Si para muy cerca de la caja, subir de 0.18 a 0.22 |

---

## Guía al llegar al robot

### PASO 0 — Entrar al entorno (2 min)

```bash
ssh root@10.42.0.1
docker start friendly_pike
docker exec -it friendly_pike bash
source /opt/ros/humble/setup.bash
```

---

### PASO 1 — Actualizar código y compilar (5 min)

```bash
cd /root/frayder_ws/src/RC3
git fetch origin && git reset --hard origin/main

# Build limpio (necesario porque cambiamos nombres de paquetes)
cd /root/frayder_ws
rm -rf build/ install/ log/
colcon build --packages-select box_detector behavior_fsm
source install/setup.bash
```

---

### PASO 2 — Encontrar cómo arranca el LiDAR (DESCONOCIDO)

El driver está comentado en nuestros launch porque no sabemos el nombre exacto.

```bash
# Ver paquetes de lidar disponibles en el robot
ros2 pkg list | grep -i lidar
ros2 pkg list | grep -i ms200
ros2 pkg list | grep -i lslidar

# Ver launches existentes en el workspace
find /root -name "*.launch.py" 2>/dev/null | head -20
```

Una vez encontrado, verificar que `/scan` llega antes de todo lo demás:

```bash
ros2 topic hz /scan   # debe dar ~10-15 Hz
```

---

### PASO 3 — Verificar orientación del LiDAR (CRÍTICO)

Poner un objeto a ~40cm **enfrente** del robot. Correr solo la FSM y ver los logs:

```bash
ros2 run behavior_fsm behavior_fsm \
  --ros-args --params-file \
  ~/frayder_ws/install/behavior_fsm/share/behavior_fsm/config/params.yaml
```

- Log dice `dist_frente ≈ 0.40` → `lidar_front_deg: 180.0` es correcto ✓
- Log dice `dist_frente = inf` → cambiar a `lidar_front_deg: 0.0`

**Cambiar si hace falta** (solo una línea en `behavior_fsm/config/params.yaml`):
```yaml
lidar_front_deg: 0.0   # cambiar de 180.0 a 0.0
```
Luego recompilar:
```bash
colcon build --packages-select behavior_fsm && source install/setup.bash
```

Alternativamente, desde el PC con `lidar_viz.py`:
```bash
python3 lidar_viz.py --front 180   # si los puntos rojos (frente) apuntan al obstáculo → OK
python3 lidar_viz.py --front 0     # probar la otra opción si no
```

---

### PASO 4 — Probar wall_follower sin autonomía

```bash
# Terminal 1: LiDAR corriendo
# Terminal 2:
ros2 run behavior_fsm wall_follower \
  --ros-args --params-file \
  ~/frayder_ws/install/behavior_fsm/share/behavior_fsm/config/params.yaml

# Terminal 3: control manual
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Mover el robot entre las paredes del jirón. Verificar que `/lateral_correction`
publica valores distintos de 0 y que el robot tiende a centrarse.

---

### PASO 5 — Probar FSM frente a una sola caja

```bash
ros2 launch behavior_fsm capytown.launch.py
```

Poner una caja a ~60cm enfrente. Los logs deben mostrar:
```
[behavior_fsm] CRUCERO → dist_frente=0.42 m
[behavior_fsm] frenando... v=0.11
[behavior_fsm] PARAR — izq=0.35m der=0.22m → rodeo IZQUIERDA
[behavior_fsm] RODEAR fase 1/3...
```

Si el robot gira **hacia** la caja en vez de alejarse → invertir la lógica en params:
cambiar `vel_giro` a negativo o intercambiar `dist_izq`/`dist_der` en el código.

---

### PASO 6 — Ajuste de parámetros (según lo visto en PASO 5)

Editar `behavior_fsm/config/params.yaml` y recompilar entre ajustes:

| Parámetro | Valor actual | Ajustar si... |
|---|---|---|
| `lidar_front_deg` | 180.0 | Robot reacciona en dirección equivocada |
| `dist_parada` | 0.18 m | Para muy cerca de la caja → subir a 0.22 |
| `angulo_rodeo_deg` | 35° | No da suficiente margen al bordear → subir |
| `avance_rodeo_seg` | 1.8 s | Se reincorpora antes de pasar la caja → subir |
| `vel_crucero` | 0.18 m/s | Va muy rápido en el jirón → bajar a 0.15 |
| `Kp` (wall_follower) | 0.80 | Oscila mucho al centrar → bajar a 0.50 |

```bash
# Recompilar tras cada cambio en params.yaml
colcon build --packages-select behavior_fsm && source install/setup.bash
```

---

### PASO 7 — Corridas completas (10 para el entregable)

**Antes de cada corrida:**

1. Medir posiciones (x, y) de las 5 cajas con cinta desde el origen del robot
2. Actualizar `box_detector/config/params.yaml`:
   ```yaml
   ground_truth: [x1, y1, x2, y2, x3, y3, x4, y4, x5, y5]
   ```
3. Recompilar: `colcon build --packages-select box_detector && source install/setup.bash`

**Corrida:**
```bash
ros2 launch behavior_fsm capytown.launch.py
# Al terminar (Ctrl+C) → se guarda metricas_lidar_YYYYMMDD_HHMMSS.csv automáticamente
```

**Copiar CSV al PC tras cada corrida:**
```bash
scp root@10.42.0.1:/root/metricas_lidar_*.csv .
```

Repetir 10 veces.

---

## Entregables pendientes

| Entregable | Estado |
|---|---|
| Paquete `box_detector` documentado | Listo |
| Paquete `behavior_fsm` (FSM mejorada) | Listo |
| `metricas_lidar.csv` con 10 corridas | Pendiente — necesita robot |
| Captura RViz con markers de cajas | Pendiente — necesita robot |
| Video compilado (5 cajas, sin tumbar) | Pendiente — necesita robot |
| Bonus (sigue-corredor o estacionamiento) | Sin definir |

---

## Comandos de referencia rápida

### Conexión y docker
```bash
ssh root@10.42.0.1
docker start friendly_pike && docker exec -it friendly_pike bash
docker stop friendly_pike
```

### Fuentes ROS2 (dentro del docker)
```bash
source /opt/ros/humble/setup.bash
source /root/frayder_ws/install/setup.bash
```

### Actualizar desde git y compilar
```bash
cd /root/frayder_ws/src/RC3
git fetch origin && git reset --hard origin/main
cd /root/frayder_ws
colcon build --packages-select box_detector behavior_fsm
source install/setup.bash
```

### Lanzar el proyecto
```bash
ros2 launch behavior_fsm capytown.launch.py   # corrida completa
ros2 launch box_detector lidar.launch.py      # solo detector (sin FSM)
```

### Monitoreo de topics
```bash
ros2 topic list
ros2 topic echo /fsm_state
ros2 topic echo /cajas_avistadas
ros2 topic echo /cmd_vel
ros2 topic echo /lateral_correction
```

### Control manual con teclado
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i=adelante  ,=atras  j=izq  l=der  k=frenar
```

### Parar el robot (emergencia)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

### Visualización LiDAR en tiempo real (desde el PC)
```bash
python3 lidar_viz.py            # frente en 180° (Yahboom por defecto)
python3 lidar_viz.py --front 0  # frente en 0°
```

### Copiar archivos del robot al PC
```bash
scp root@10.42.0.1:/root/metricas_lidar_*.csv .
scp root@10.42.0.1:/root/frayder_ws/src/RC3/*.png .
```
