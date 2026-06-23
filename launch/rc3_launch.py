from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('capytown_esan'),
        'config', 'params.yaml'
    )

    box_detector = Node(
        package='capytown_esan',
        executable='box_detector',
        name='box_detector',
        parameters=[params],
        output='screen',
    )

    behavior_fsm = Node(
        package='capytown_esan',
        executable='behavior_fsm',
        name='behavior_fsm',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([box_detector, behavior_fsm])
