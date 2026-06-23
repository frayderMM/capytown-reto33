# Contexto RC2 → RC3

## Robot
- **IP:** `10.42.0.1`
- **Usuario:** `root`
- **Hardware:** Yahboom Pi5 (Raspberry Pi 5 + ESP32-S3 microROS)
- **ROS2:** Humble

## Workspace en el robot
```bash
# Actualizar y compilar
cd /root/frayder_ws/src/capytown_esan && git fetch origin && git reset --hard origin/main
cd /root/frayder_ws && colcon build --packages-select capytown_esan && source install/setup.bash
```

## Flujo de trabajo
1. Editar archivos en el **PC (Windows)**
2. `git add` + `git commit` + `git push origin main`
3. En el robot: pull + build (comandos arriba)
4. Probar en pista

## Copiar archivos del robot al PC
```bash
scp root@10.42.0.1:"/root/frayder_ws/src/capytown_esan/*.png" .
```

## Grabar bag y generar plots
```bash
# Grabar (en el robot)
ros2 bag record /lane_error /odom_raw /cmd_vel -o ~/bags/s11_final

# Generar plots (en el robot)
cd /root/frayder_ws/src/capytown_esan
python3 scripts/plot_lane_error.py ~/bags/s11_final
python3 scripts/plot_trajectory.py ~/bags/s11_final
```

## Repo GitHub
- **URL:** https://github.com/frayderMM/Line_Detector_RosCar_Pi_R2
- **Rama principal:** `main`

## Lo aprendido en RC2 (lane following)
- HSV es más robusto que RGB para detectar líneas bajo iluminación variable
- `yellow_setpoint` bajo (0.26–0.28) = robot más a la derecha, lejos del amarillo
- Conteo de curvas por `|ω| > 1.5 rad/s` es más fiable que detectar si desaparece la línea blanca
- `kff` (feed-forward) ayuda a anticipar curvas, pero si es muy alto causa deriva post-curva
- Siempre tuning en orden: primero `kp`, luego `kd`, al final `ki` (si hace falta)
- `px_per_meter=600` es calibración aproximada del IPM — los valores absolutos de error_m no son exactos, pero el patrón sí es válido
