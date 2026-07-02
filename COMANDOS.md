# Comandos rápidos — CapyTown Reto 3 (Guardian v4: RC3 + RETROCESO + heading)

## 1 — Entrar al robot
```bash
ssh pi@10.42.0.1
sudo docker ps                     # el nombre del contenedor CAMBIA cada sesión
sudo docker exec -it <contenedor> bash
```

## 2 — Pull, build (una sola vez por cambio de código)
IMPORTANTE: compilar `box_detector` y `behavior_fsm` **juntos** — si se
compila solo `behavior_fsm`, el `box_detector` instalado queda con el
build viejo aunque el código fuente ya esté actualizado (censo sin tope,
por ejemplo).
```bash
cd /root/yahboomcar_ws/src/capytown-reto33 && git fetch origin && git reset --hard origin/test
cd /root/yahboomcar_ws && colcon build --packages-select box_detector behavior_fsm --symlink-install && source install/setup.bash
```

## 3 — Bringup (terminal A, antes que nada)
```bash
sudo docker exec -it <contenedor> bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch capytown_esan bringup.launch.py
```

## 4 — TERMINAL 1: correr el reto (censo + guardián + wall_follower)
```bash
sudo docker exec -it <contenedor> bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch behavior_fsm capytown.launch.py
```

## 5 — TERMINAL 2: pantalla (LiDAR + obstáculos clasificados + recorrido)
```bash
sudo docker exec -it <contenedor> bash
source /root/yahboomcar_ws/install/setup.bash
python3 /root/yahboomcar_ws/src/capytown-reto33/lidar_viz.py
```

## Arquitectura actual (para la defensa)
- **Estados**: `CRUCERO → GIRO → RODEO → CRUCERO`, y desde cualquier
  estado `→ RETROCESO → CRUCERO` si hay un choque real (STOP absoluto) o
  si `GIRO` no encuentra salida a tiempo (espacio muy chico) — en vez de
  quedarse congelado o seguir girando/avanzando a ciegas, retrocede una
  distancia fija girando hacia el lado con más espacio.
- **`wall_follower.py`** (nodo aparte, dentro del paquete `behavior_fsm`)
  sigue la pared derecha por RANSAC — robusto a que una caja interrumpa el
  tramo de pared (los puntos de la caja quedan fuera del modelo como
  outliers), a diferencia de Split-and-Merge que necesita puntos
  contiguos.
- **Tracking de crucero con heading**: `_w_der_pd()` no usa solo la
  distancia RANSAC — toma el segmento `PARED` ya clasificado por
  `percepcion.py` (borde exterior) y le suma un término de ángulo
  (`K_alpha * alpha`) para evitar el zigzag de un PD de una sola
  variable; si no hay segmento clasificado disponible, cae a RANSAC sin
  heading.
- **Clasificación caja/pared** (`percepcion.py`): por tamaño — un cluster
  que cabe en el tamaño de una caja real (≤ `lado_caja_max`) es `CAJA`;
  más grande es `PARED`. Cada LÍNEA individual en el panel de diagnóstico
  se pinta por su propio largo (≤ `lado_caja_linea` → naranja/caja, mayor
  → azul/pared), no por la clase del cluster completo.
- **STOP de seguridad**: `dist_frente`/`dist_izq_raw`/`dist_der_raw` se
  miden crudo desde el LiDAR en `behavior_fsm.py` (no dependen de
  RANSAC), y **si tocan un umbral, el robot SÍ retrocede** (no es cierto
  que "nunca retrocede" — eso era de una versión anterior).
- Rama de trabajo: **`test`** (no `main`).

## Posición de inicio del robot
- Corredor **sur**, esquina **suroeste**, apuntando al **este**
- Pegado a la derecha: lado derecho a ~15 cm de la pared sur
