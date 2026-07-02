import os
from glob import glob
from setuptools import setup

package_name = 'box_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frayderMM',
    maintainer_email='fraydermezamorveli@gmail.com',
    description='Parte A del reto CapyTown: censo de cajas con LiDAR 2D.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'box_detector = box_detector.box_detector:main',
            'metrics_logger = box_detector.metrics_logger:main',
        ],
    },
)
