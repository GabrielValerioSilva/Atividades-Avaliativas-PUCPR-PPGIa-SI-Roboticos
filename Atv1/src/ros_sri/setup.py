from setuptools import find_packages, setup

# Nome do pacote ROS 2 usado também no índice do ament e nos comandos ros2 run.
package_name = 'ros_sri'

setup(
    name=package_name,
    version='0.0.0',
    # Descobre o módulo Python ros_sri e exclui os testes do pacote instalado.
    packages=find_packages(exclude=['test']),
    # Registra o pacote no ament e instala o manifesto package.xml.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # setuptools é a dependência de empacotamento; zip_safe permite instalação compactada.
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
# Cada console_script cria um comando ros2 run ligado à função main do nó indicado.
entry_points={
        'console_scripts': [
            'pose_subscriber = ros_sri.pose_subscriber:main',
            'square_publisher = ros_sri.square_publisher:main',
            'position_controller = ros_sri.position_controller:main',
        ],
    },
)
