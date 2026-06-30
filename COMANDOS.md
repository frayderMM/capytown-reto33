# Comandos rápidos — CapyTown Reto 3 (Frayder)

## 1 — Entrar al robot
```bash
ssh pi@10.42.0.1
sudo docker exec -it friendly_pike bash
```

## 2 — Primera vez (clonar en el robot)
```bash
cd /root/yahboomcar_ws/src
git clone https://github.com/frayderMM/capytown-reto33.git capytown-reto33
cd /root/yahboomcar_ws && colcon build --packages-select box_detector behavior_fsm && source install/setup.bash
```

## 3 — Pull, build y correr FSM (terminal B)
```bash
cd /root/yahboomcar_ws/src/capytown-reto33 && git fetch origin && git reset --hard origin/main
cd /root/yahboomcar_ws && colcon build --packages-select box_detector behavior_fsm && source install/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch behavior_fsm capytown.launch.py
```

## 4 — Bringup (terminal A, antes que la FSM)
```bash
sudo docker exec -it friendly_pike bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch capytown_esan bringup.launch.py
```

## 5 — Mapa y visor (aparte, opcional)

**Terminal C — mapa**
```bash
sudo docker exec -it friendly_pike bash
source /root/yahboomcar_ws/install/setup.bash
python3 /root/yahboomcar_ws/src/capytown-reto33/map_builder.py
```

**Terminal D — visor LiDAR**
```bash
sudo docker exec -it friendly_pike bash
source /root/yahboomcar_ws/install/setup.bash
python3 /root/yahboomcar_ws/src/capytown-reto33/lidar_viz.py
```

## DNS (solo si falla `git fetch`/`clone` por resolución de nombres)
```bash
echo "nameserver 8.8.8.8" > /etc/resolv.conf
```

## Posición de inicio del robot
- Corredor **sur**, esquina **suroeste**
- Apuntando al **este**
- Centrado: 30 cm de la pared sur y 30 cm de la isla
