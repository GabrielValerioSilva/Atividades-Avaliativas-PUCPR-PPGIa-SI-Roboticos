from setuptools import find_packages, setup

# Nome registrado no ROS 2 e usado como prefixo dos executáveis do pacote.
package_name = 'turtlebot_lidar'

setup(
    name=package_name,
    version='0.0.0',
    # Localiza os módulos Python do pacote sem incluir a pasta de testes.
    packages=find_packages(exclude=['test']),
    # Registra o pacote no ament e disponibiliza seu manifesto após a instalação.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel_valerio',
    maintainer_email='gabriel_valerio@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
# Cria os comandos ros2 run para visualização do LiDAR e navegação reativa.
entry_points={
        'console_scripts': [
            'lidar_plotter = turtlebot_lidar.lidar_plotter:main',
            'reactive_navigation = turtlebot_lidar.reactive_navigation:main',
        ],
    },
)
