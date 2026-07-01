# CapyTown RC3 (version corregida) - ESAN Robotica de Moviles 2026-I

Robot Yahboom Pi5 - LiDAR MS200 - ROS2 Humble

---

## Estructura

```
RC3/
bbox_detector/        # Parte A: Censo de cajas con LiDAR
b   box_detector/
b   b   box_detector.py    # Nodo: detecta y cuenta cajas
b   b   lidar_utils.py     # Funciones puras de procesamiento LiDAR
b   b   metrics_logger.py  # Nodo: guarda VP/FP/FN en CSV
b   config/params.yaml
b   launch/lidar.launch.py

behavior_fsm/        # Parte B: Guardian con FSM
b   behavior_fsm/
b   b   behavior_fsm.py    # FSM 3 estados: CRUCERO/PARAR/RODEAR
b   b   wall_follower.py   # Centrado lateral Split-and-Merge + PD
b   config/params.yaml
b   launch/capytown.launch.py

lidar_viz.py         # Monitor visual matplotlib (corre en robot por VNC)
```

---

## Instalacion en el robot (primera vez)

```bash
# Dentro del docker friendly_pike
cd /root/frayder_ws/src
git clone https://github.com/frayderMM/capytown-esan-rc3.git RC3

# Dependencias para lidar_viz.py
sudo apt install python3-tk python3-matplotlib

cd /root/frayder_ws
colcon build --packages-select box_detector behavior_fsm
source install/setup.bash
```

## Actualizar desde git (entre sesiones)

```bash
cd /root/frayder_ws/src/RC3
git fetch origin && git reset --hard origin/main
cd /root/frayder_ws
colcon build --packages-select box_detector behavior_fsm
source install/setup.bash
```

---

## Lanzar

```bash
# Corrida completa (Parte A + Parte B)
ros2 launch behavior_fsm capytown.launch.py

# Solo detector de cajas (debug)
ros2 launch box_detector lidar.launch.py

# Monitor visual LiDAR (en terminal aparte, dentro del VNC)
python3 /root/frayder_ws/src/RC3/lidar_viz.py
```

---

## Flujo entre corridas (sin rebuild)

El launch usa el yaml INSTALADO, no el fuente. Para cambiar ground_truth
entre corridas sin perder 2 minutos en colcon build:

```bash
# Editar el yaml instalado directamente
nano /root/frayder_ws/install/box_detector/share/box_detector/config/params.yaml
# Cambiar la linea ground_truth: [x1,y1, x2,y2, x3,y3, x4,y4, x5,y5]
# Guardar y relanzar - S
```

Solo hace falta colcon build cuando se cambia CODIGO Python (no yamls).

El CSV de metricas se acumula en `/root/metricas_lidar.csv`.
Cada Ctrl+C agrega una fila nueva. Completar colisiones y rodeo_exitoso a mano.

---

## Parametros clave

**Orientacion del LiDAR** - si el robot reacciona al reves:
```yaml
# En el yaml instalado:
lidar_front_deg: 180.0   # Yahboom MS200 por defecto
# lidar_front_deg: 0.0   # si el frente esta en 0 grados
```

**Ground-truth** - posiciones reales de las 5 cajas en odom (medir con cinta):
```yaml
ground_truth: [x1, y1, x2, y2, x3, y3, x4, y4, x5, y5]
```
