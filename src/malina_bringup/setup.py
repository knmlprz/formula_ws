import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rc_car_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launche (XML)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
        # Parametry
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Opis robota (URDF/xacro)
        (os.path.join('share', package_name, 'description', 'urdf'),
            glob('description/urdf/*')),
        # Konfiguracja RViz
        (os.path.join('share', package_name, 'description', 'rviz'),
            glob('description/rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='msurowka',
    maintainer_email='msurowka@example.com',
    description='Sterownik pojazdu RC (Ackermann) dla ROS2 Jazzy: '
                'cmd_vel -> serwo + ODrive (CAN), odometria, TF, E-Stop.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_controller = rc_car_controller.cmd_vel_controller:main',
        ],
    },
)
