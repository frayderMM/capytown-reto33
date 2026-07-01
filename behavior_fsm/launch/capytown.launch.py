"""
capytown.launch.py — Lanzamiento COMPLETO del reto.

    ros2 launch behavior_fsm capytown.launch.py

Lanza: box_detector (Parte A) + metrics_logger + guardián (Parte B).
El guardián ya integra el seguimiento de pared derecha; wall_follower
es solo un nodo de depuración opcional y NO se lanza aquí.
El bringup del robot (driver LiDAR + base) va aparte:
    ros2 launch capytown_esan bringup.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_det = os.path.join(
        get_package_share_directory('box_detector'), 'config', 'params.yaml')
    params_fsm = os.path.join(
        get_package_share_directory('behavior_fsm'), 'config', 'params.yaml')

    detector = Node(
        package='box_detector',
        executable='box_detector',
        name='box_detector',
        output='screen',
        parameters=[params_det],
    )

    metrics = Node(
        package='box_detector',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[params_det],
    )

    guardian = Node(
        package='behavior_fsm',
        executable='behavior_fsm',
        name='behavior_fsm',
        output='screen',
        parameters=[params_fsm],
    )

    return LaunchDescription([
        detector,
        metrics,
        guardian,
    ])
