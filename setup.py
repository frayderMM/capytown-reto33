from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'capytown_esan'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frayderMM',
    maintainer_email='fraydermezamorveli@gmail.com',
    description='CapyTown RC3 — LiDAR box detection and FSM guardian',
    license='MIT',
    entry_points={
        'console_scripts': [
            'box_detector = capytown_esan.box_detector:main',
            'behavior_fsm  = capytown_esan.behavior_fsm:main',
        ],
    },
)
