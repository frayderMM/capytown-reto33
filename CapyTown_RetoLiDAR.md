# CapyTown Reto LiDAR

**RETO CLASIFICATORIO — LiDAR**

El Censo y el Guardián de las Caja

*Procesamiento de LaserScan, segmentación por clustering y reacción ante obstáculos*

| **Curso**      | Robótica de Móviles — ESAN 2026-I             |
|----------------|-----------------------------------------------|
| **Proyecto**   | CapyTown — ciudad andina autónoma             |
| **Plataforma** | Yahboom + Raspberry Pi 5 · LiDAR MS200 · ROS2 |
| **Escenario**  | El Jirón Principal                            |
| **Sensor**     | LiDAR 2D — solo /scan                         |
| **Semana**     | Penúltima — antecede al Gran Qhapaq Ñan       |
| **Evaluación** | **40% Implementación · 60% Defensa**          |

*En CapyTown, las cajas del mercado a veces quedan olvidadas en medio del jirón después de la feria. El robot no las esquiva por suerte: las “ve” con su LiDAR, cuenta cuántas hay, anota dónde quedaron y las rodea con paciencia andina sin tumbar ni una.*

**Contenido**

[1. Objetivo del reto [1](#objetivo-del-reto)](#objetivo-del-reto)

[Objetivos de aprendizaje [1](#objetivos-de-aprendizaje)](#objetivos-de-aprendizaje)

[2. Contexto técnico: ¿por qué cajas? [1](#contexto-técnico-por-qué-cajas)](#contexto-técnico-por-qué-cajas)

[3. Fundamento técnico [1](#fundamento-técnico)](#fundamento-técnico)

[3.1 Anatomía del LaserScan [1](#anatomía-del-laserscan)](#anatomía-del-laserscan)

[3.2 De polar a cartesiano y a odom [1](#de-polar-a-cartesiano-y-a-odom)](#de-polar-a-cartesiano-y-a-odom)

[3.3 Filtrado y segmentación por clustering 1D [1](#filtrado-y-segmentación-por-clustering-1d)](#filtrado-y-segmentación-por-clustering-1d)

[3.4 Sector de seguridad y FSM de reacción [1](#sector-de-seguridad-y-fsm-de-reacción)](#sector-de-seguridad-y-fsm-de-reacción)

[4. Descripción del reto de laboratorio [1](#descripción-del-reto-de-laboratorio)](#descripción-del-reto-de-laboratorio)

[Parte A — El Censo (percepción) [1](#parte-a-el-censo-percepción)](#parte-a-el-censo-percepción)

[Parte B — El Guardián (reacción) [1](#parte-b-el-guardián-reacción)](#parte-b-el-guardián-reacción)

[5. Requisitos técnicos (ROS2) [1](#requisitos-técnicos-ros2)](#requisitos-técnicos-ros2)

[6. Métricas por corrida (metricas_lidar.csv) [1](#métricas-por-corrida-metricas_lidar.csv)](#métricas-por-corrida-metricas_lidar.csv)

[7. Entregables [1](#entregables)](#entregables)

[8. Cronograma sugerido (sesión de laboratorio) [1](#cronograma-sugerido-sesión-de-laboratorio)](#cronograma-sugerido-sesión-de-laboratorio)

[9. Rúbrica de evaluación [1](#rúbrica-de-evaluación)](#rúbrica-de-evaluación)

[10. Preguntas guía para la defensa [1](#preguntas-guía-para-la-defensa)](#preguntas-guía-para-la-defensa)

[11. Impacto en el Gran Reto [1](#impacto-en-el-gran-reto)](#impacto-en-el-gran-reto)

# 1. Objetivo del reto

El equipo debe dotar al robot de la capacidad de percibir cajas de cartón usando únicamente el LiDAR 2D y reaccionar de forma segura ante ellas mientras recorre El Jirón Principal. El reto integra dos capacidades que se desarrollan en simultáneo:

- **Censar (percepción):** detectar, contar y ubicar las cajas a partir de la nube de puntos del /scan.

- **Guardar (reacción):** detenerse a tiempo y rodear cada caja sin tumbarla ni rozarla.

Es la última estación técnica antes del Gran Qhapaq Ñan: el nodo de detección y el estado de rodeo se reutilizan tal cual en el Gran Reto, sin reprogramar percepción.

## Objetivos de aprendizaje

- Interpretar la geometría del LiDAR 2D y la estructura del mensaje sensor_msgs/LaserScan.

- Segmentar una nube de puntos 1D por discontinuidad de rango (clustering) y validar candidatos por ancho y número de puntos.

- Transformar coordenadas polares a cartesianas y componer la pose de /odom para fijar la caja en el marco del mundo.

- Diseñar una máquina de estados reactiva y cuantificar el desempeño con métricas (VP, FP, FN, tasa de detección).

# 2. Contexto técnico: ¿por qué cajas?

Las cajas de cartón son el blanco ideal para un sistema LiDAR-only: caras planas y verticales, bordes y esquinas bien definidos, y altura constante a la del plano de barrido del MS200. Esto produce retornos del /scan nítidos y continuos sobre cada cara, lo que hace viable la segmentación por clustering — cada caja es un grupo compacto de puntos con rango similar. Un objeto blando, irregular o de baja reflectividad daría una nube ruidosa y rompería la técnica central del reto.

**Montaje recomendado:** cajas de ~15–25 cm de lado, separadas entre sí al menos 40 cm (para que el clustering las distinga), colocadas dentro del jirón sin bloquear por completo el carril.

# 3. Fundamento técnico

## 3.1 Anatomía del LaserScan

El mensaje entrega angle_min, angle_max, angle_increment, range_min, range_max y el vector ranges\[\]. El índice i del arreglo corresponde al ángulo θ = angle_min + i · angle_increment.

## 3.2 De polar a cartesiano y a odom

Para cada punto: x = r·cos(θ), y = r·sin(θ) en el marco base_link. Luego se compone con la pose (x_r, y_r, yaw) de /odom para llevar el centroide al marco odom, de modo que la caja queda fija en el mundo aunque el robot se mueva.

## 3.3 Filtrado y segmentación por clustering 1D

- Filtrado previo: descartar inf/nan y lecturas fuera de \[range_min, range_max\].

- Clustering: recorrer los puntos ordenados por ángulo y abrir un cluster nuevo cuando el salto de rango entre puntos consecutivos supera un umbral (p. ej. 8–15 cm).

- Validación: cada cluster con número de puntos y ancho aparente dentro de lo esperado para una caja se acepta como candidato; el resto (paredes, ruido) se rechaza.

## 3.4 Sector de seguridad y FSM de reacción

Zona de alerta frontal a \< 30 cm en ±45°; criterio de parada a ≥ 15 cm de la caja. La reacción se modela como una máquina de estados:

**CRUCERO → CAJA_DETECTADA → PARAR → ESPERAR_3S → RODEAR → CRUCERO**

# 4. Descripción del reto de laboratorio

El robot recorre El Jirón Principal con 5 cajas en posiciones aleatorias dentro del jirón. Se realizan 10 corridas (las posiciones cambian entre corridas). Cada corrida tiene dos partes simultáneas:

## Parte A — El Censo (percepción)

- Nodo box_detector suscrito a /scan: filtra y segmenta la nube en clusters.

- Para cada cluster válido calcula el centroide (x, y) y lo transforma al marco odom.

- Publica en /cajas_avistadas la lista de cajas detectadas con su posición estimada, y las visualiza como Marker en RViz.

- El docente valida con cinta métrica si cada posición reportada está a ≤ 30 cm de la caja real.

## Parte B — El Guardián (reacción)

- Nuevo estado en behavior_fsm: CAJA_EN_PISTA → PARAR → ESPERAR_3S → RODEAR.

- Detención a ≥ 15 cm de la caja — “no se tumba la mercadería del vecino”.

- Tras 3 s, maniobra de rodeo: giro ~30° + avance + giro ~−30° para reincorporarse al carril.

- Objetivo de la corrida: recorrer el jirón completo sin tumbar ni rozar ninguna caja.

# 5. Requisitos técnicos (ROS2)

La solución se organiza en dos paquetes: box_detector (Parte A) y behavior_fsm (Parte B). Tópicos involucrados:

| **Tópico**       | **Tipo**                       | **Dir.** | **Nodo**                   |
|------------------|--------------------------------|----------|----------------------------|
| /scan            | sensor_msgs/LaserScan          | sub      | box_detector, behavior_fsm |
| /odom            | nav_msgs/Odometry              | sub      | box_detector               |
| /cajas_avistadas | geometry_msgs/PoseArray        | pub      | box_detector               |
| /cajas_markers   | visualization_msgs/MarkerArray | pub      | box_detector               |
| /cmd_vel         | geometry_msgs/Twist            | pub      | behavior_fsm               |

# 6. Métricas por corrida (metricas_lidar.csv)

| **Campo**          | **Descripción**                                          |
|--------------------|----------------------------------------------------------|
| corrida            | Número de corrida (1–10)                                 |
| cajas_reales       | Cuántas cajas se colocaron (normalmente 5)               |
| cajas_detectadas   | Cuántas reportó en /cajas_avistadas                      |
| VP / FP / FN       | Verdaderos positivos, falsos positivos, falsos negativos |
| error_pos_prom_cm  | Error promedio de posición de los VP (cm)                |
| dist_min_parada_cm | Distancia mínima al detenerse frente a una caja          |
| colisiones         | Número de cajas rozadas o tumbadas                       |
| rodeo_exitoso      | Sí/No por caja que requirió rodeo                        |

*Tasa de detección = VP / (VP + FN). Es la métrica central del censo y debe reportarse agregada sobre las 10 corridas.*

# 7. Entregables

Repositorio Git: capytown_G\<n\>\_lidar

- Paquete box_detector (segmentación + centroides + publicación) documentado.

- Estado nuevo en behavior_fsm (parar / esperar / rodear).

- metricas_lidar.csv con las 10 corridas.

- Captura de RViz mostrando los clusters/markers de las cajas.

- Video compilado: las 5 cajas en posiciones distintas, robot que las censa y no tumba ninguna.

- **Bonus (elegir uno):** Estacionamiento “El Tambo” — detenerse perpendicular a la cara de una caja a 10 cm ±2; o Sigue-corredor — mantenerse centrado entre dos filas de cajas usando distancias laterales del /scan.

# 8. Cronograma sugerido (sesión de laboratorio)

| **Tiempo** | **Actividad**                                                                 | **Producto**                |
|------------|-------------------------------------------------------------------------------|-----------------------------|
| 5 min      | Activación cognitiva: dibujar sector de vigilancia y criterio de segmentación | Diagrama en papel           |
| 1.5 h      | Teoría: LaserScan, clustering, transformaciones, FSM                          | Notas / pseudocódigo        |
| 2–3 h      | Implementación y 10 corridas                                                  | Código + metricas_lidar.csv |
| 5 min      | Cierre reflexivo y preparación de la defensa                                  | Conclusiones                |

# 9. Rúbrica de evaluación

La nota se reparte en dos componentes: la Implementación (lo que el robot logra hacer) pesa 40% y la Defensa (lo que el equipo demuestra comprender) pesa 60%. Se privilegia la comprensión y la justificación por encima del resultado puro: un sistema imperfecto bien entendido y bien argumentado puede obtener una nota alta.

**Componente 1 — Implementación (40%)**

| **Criterio**                                                                                        | **Peso** | **Evidencia / Logro**                                              |
|-----------------------------------------------------------------------------------------------------|----------|--------------------------------------------------------------------|
| Censo funcional (Parte A): detecta y reporta las cajas por clustering y publica en /cajas_avistadas | **12%**  | Tasa de detección ≥ 80% sobre las 10 corridas                      |
| Precisión del censo (Parte A): posición correcta de los VP                                          | **10%**  | Error promedio ≤ 30 cm (cinta métrica) y FP = 0 en ≥ 8/10 corridas |
| Guardián seguro (Parte B): se detiene y rodea sin dañar las cajas                                   | **13%**  | 0 colisiones en las 10 corridas y parada ≥ 15 cm en promedio       |
| Calidad técnica y bonus: código ordenado/documentado, registro de métricas y reto bonus operativo   | **5%**   | CSV completo + RViz + bonus elegido funcional                      |
| **Subtotal Implementación**                                                                         | **40%**  |                                                                    |

**Componente 2 — Defensa (60%)**

| **Criterio**                               | **Peso** | **Qué se evalúa**                                                                                    |
|--------------------------------------------|----------|------------------------------------------------------------------------------------------------------|
| Comprensión teórica                        | **18%**  | Explica LaserScan, clustering 1D y las transformaciones polar→cartesiano→odom con corrección         |
| Justificación de decisiones de diseño      | **16%**  | Argumenta umbral de salto, sector de vigilancia y el balance FP vs FN con criterio, no por intuición |
| Análisis de métricas y resultados          | **14%**  | Interpreta VP/FP/FN, compara corridas y propone mejoras basadas en datos                             |
| Dominio del código y respuesta a preguntas | **8%**   | Ubica y explica cualquier parte del código; responde repreguntas “qué pasaría si…”                   |
| Comunicación y trabajo de equipo           | **4%**   | Claridad, uso del tiempo y participación equilibrada de todos los integrantes                        |
| **Subtotal Defensa**                       | **60%**  |                                                                                                      |

**Niveles de desempeño (aplican a cada criterio)**

| **Nivel**                | **Descriptor**                                                                          |
|--------------------------|-----------------------------------------------------------------------------------------|
| **Excelente (100%)**     | Cumple el criterio por completo, con precisión y profundidad; anticipa casos límite.    |
| **Aceptable (60–80%)**   | Cumple lo esencial con pequeños vacíos o imprecisiones que no comprometen el resultado. |
| **Insuficiente (\<40%)** | Vacíos de fondo: no logra el objetivo o no puede justificar sus decisiones.             |

# 10. Preguntas guía para la defensa

El jurado puede formular, entre otras, las siguientes preguntas. Conviene que el equipo las prepare por anticipado:

1.  ¿Qué umbral de salto de rango usaron para el clustering y por qué? ¿Qué pasó con dos cajas separadas por menos de 40 cm?

2.  En el censo, ¿es peor un falso positivo o un falso negativo? ¿Cómo afectó eso sus decisiones de umbral?

3.  ¿Cómo transforman el centroide de base_link a odom y por qué es necesario para fijar la caja en el mundo?

4.  ¿Por qué el LiDAR solo no puede distinguir una caja de una pared o una persona? ¿Qué sensor agregarían?

5.  ¿Qué cambiarían para que el sistema funcione con el doble de cajas o con cajas más pequeñas?

# 11. Impacto en el Gran Reto

- Puntaje completo → +1 crédito en la ronda de percepción (“El Censo de Cajas”) del Gran Qhapaq Ñan, hasta un máximo de 5 pts en esa ronda.

- El nodo box_detector y el estado de rodeo de la FSM se reutilizan tal cual en el Gran Reto: no se programa percepción nueva, solo se integra con lane following y odometría.
