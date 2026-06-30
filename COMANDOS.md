# Comandos rápidos — CapyTown Reto 3 (Frayder)

## 1 — Entrar al robot
```bash
ssh yahboom@10.42.0.1
docker exec -it friendly_pike bash
```

## 2 — Primera vez (clonar)
```bash
echo "nameserver 8.8.8.8" > /etc/resolv.conf
mkdir -p /root/frayder_ws/src/capytown-reto33
git clone https://github.com/frayderMM/capytown-reto33.git /root/frayder_ws/src/capytown-reto33
cd /root/frayder_ws && colcon build --packages-select behavior_fsm box_detector && source install/setup.bash
```

## 3 — Pull y build (siguientes veces)
```bash
echo "nameserver 8.8.8.8" > /etc/resolv.conf
cd /root/frayder_ws/src/capytown-reto33 && git fetch origin && git reset --hard origin/main
cd /root/frayder_ws && colcon build --packages-select behavior_fsm box_detector && source install/setup.bash
```

## 4 — Correr el robot (Terminal A)
```bash
ros2 launch behavior_fsm capytown.launch.py
```

## 5 — Correr el visualizador (Terminal B)
```bash
docker exec -it friendly_pike bash
source /root/frayder_ws/install/setup.bash
python3 /root/frayder_ws/src/capytown-reto33/map_builder.py
```

## Posición de inicio del robot
- Corredor **sur**, esquina **suroeste**
- Apuntando al **este**
- Centrado: 30 cm de la pared sur y 30 cm de la isla
