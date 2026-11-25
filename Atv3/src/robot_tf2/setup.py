import os
from glob import glob
from setuptools import find_packages, setup

# Nome do pacote ROS 2 usado para instalação e descoberta dos nós.
package_name = 'robot_tf2'

setup(
    name=package_name,
    version='0.0.0',
    # Inclui os módulos Python do pacote, excluindo os testes.
    packages=find_packages(exclude=['test']),
    # Instala manifesto, marcador ament e todos os launch files do diretório launch.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel_valerio',
    maintainer_email='gabriel@pucpr.br',
    description='TF2 e mapeamento global em odom',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Registra os comandos ros2 run para mapeamento TF2 e navegação reativa.
    entry_points={
        'console_scripts': [
            'tf2_lidar_mapper = robot_tf2.tf2_lidar_mapper:main',
            'reactive_navigation = robot_tf2.reactive_navigation:main',
        ],
    },
)