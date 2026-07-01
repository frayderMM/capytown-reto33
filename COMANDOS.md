# Comandos rápidos — CapyTown Reto 3 (versión corregida)

## 1 — Entrar al robot
```bash
ssh pi@10.42.0.1
sudo docker exec -it friendly_pike bash
```

## 2 — Pull, build (una sola vez por cambio de código)
```bash
cd /root/yahboomcar_ws/src/capytown-reto33 && git fetch origin && git reset --hard origin/main
cd /root/yahboomcar_ws && colcon build --packages-select box_detector behavior_fsm && source install/setup.bash
```

## 3 — Bringup (terminal A, antes que nada)
```bash
sudo docker exec -it friendly_pike bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch capytown_esan bringup.launch.py
```

## 4 — TERMINAL 1: correr el reto (censo + guardián)
```bash
sudo docker exec -it friendly_pike bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch behavior_fsm capytown.launch.py
```

## 5 — TERMINAL 2: pantalla (LiDAR + obstáculos clasificados + recorrido)
```bash
sudo docker exec -it friendly_pike bash
source /root/yahboomcar_ws/install/setup.bash
python3 /root/yahboomcar_ws/src/capytown-reto33/lidar_viz.py
```

## Qué cambió en esta versión (para la defensa)
- **Marco del LiDAR unificado**: el MS200 va con el frente en raw=180°;
  antes solo la FSM lo rotaba — wall_follower y box_detector procesaban el
  scan crudo (izq/der invertidas, censo espejado). Ahora TODO se rota a
  base_link en `behavior_fsm/percepcion.py` y `box_detector/lidar_utils.py`.
- **Bug del Split-and-Merge corregido**: `_split(der)[1:]` descartaba una
  de las dos paredes de cada esquina — por eso "caja vs esquina era poco
  confiable". Corregido, la clasificación por lados vuelve a funcionar:
  CAJA = todos los lados ≤ 32 cm; ESQUINA = dos lados ≥ 45 cm
  perpendiculares; PARED = un lado ≥ 45 cm. Además, voto por mayoría
  sobre los últimos 5 barridos (un scan ruidoso no decide).
- **FSM del reto restaurada**: CRUCERO → CAJA_DETECTADA → PARAR (≥15 cm)
  → ESPERAR_3S → RODEAR (+45°, diagonal, −45°, recto) → CRUCERO; y
  GIRAR_ESQUINA (90° izquierda, lazo antihorario, con cooldown).
- **Un solo nodo de control**: el guardián integra el seguimiento de la
  pared derecha (PD + alineación + limitador de tasa de cambio anti-zigzag,
  velocidad adaptativa). Sin carreras por /lateral_correction.
- **Footprint real** (15/10/8 cm) en todos los umbrales + emergencia
  omnidireccional (excluye el cono trasero del cable) + nunca retrocede.
- Giros y avances medidos con **/odom_raw** (fallback por tiempo).

## Posición de inicio del robot
- Corredor **sur**, esquina **suroeste**, apuntando al **este**
- Pegado a la derecha: lado derecho a ~15 cm de la pared sur
