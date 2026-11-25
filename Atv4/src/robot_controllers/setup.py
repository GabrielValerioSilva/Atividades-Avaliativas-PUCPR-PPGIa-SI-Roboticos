from setuptools import find_packages, setup

# Nome de pacote usado pelo colcon e pelos comandos ros2 run.
package_name = 'robot_controllers'

setup(
    name=package_name,
    version='0.0.0',
    # Localiza os módulos dos três controladores sem instalar os testes.
    packages=find_packages(exclude=['test']),
    # Registra o pacote no ament e instala o manifesto ROS 2.
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
# Expõe cada controlador como um executável ros2 run independente.
entry_points={
        'console_scripts': [
            'pid_controller = robot_controllers.pid_controller:main',
            'fuzzy_controller = robot_controllers.fuzzy_controller:main',
            'hybrid_controller = robot_controllers.hybrid_controller:main',
        ],
    },
)
