# Comandos rápidos — CapyTown Reto 3 (Frayder)

## 1 — Entrar al robot
```bash
ssh pi@10.42.0.1
sudo docker exec -it friendly_pike bash
```

## 2 — Primera vez (clonar en el robot)
```bash
echo "nameserver 8.8.8.8" > /etc/resolv.conf
cd /root/yahboomcar_ws/src
git clone https://github.com/frayderMM/capytown-reto33.git capytown-reto33
cd /root/yahboomcar_ws && colcon build --packages-select box_detector behavior_fsm && source install/setup.bash
```

## 3 — Pull y build (siguientes veces)
```bash
echo "nameserver 8.8.8.8" > /etc/resolv.conf
cd /root/yahboomcar_ws/src/capytown-reto33 && git fetch origin && git reset --hard origin/main
cd /root/yahboomcar_ws && colcon build --packages-select box_detector behavior_fsm && source install/setup.bash
```

## 4 — Correr (3 terminales)

**Terminal A — bringup (LiDAR + base + odometría)**
```bash
sudo docker exec -it friendly_pike bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch capytown_esan bringup.launch.py
```

**Terminal B — FSM (comportamiento del robot)**
```bash
sudo docker exec -it friendly_pike bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch behavior_fsm capytown.launch.py
```

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

## Posición de inicio del robot
- Corredor **sur**, esquina **suroeste**
- Apuntando al **este**
- Centrado: 30 cm de la pared sur y 30 cm de la isla
