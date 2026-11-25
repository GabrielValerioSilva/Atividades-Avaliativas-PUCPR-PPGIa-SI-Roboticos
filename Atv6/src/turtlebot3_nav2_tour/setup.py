from setuptools import find_packages, setup

# Nome compartilhado entre o pacote ament e o comando ros2 run.
package_name = 'turtlebot3_nav2_tour'

setup(
    name=package_name,
    version='0.1.0',
    # Localiza os módulos Python do nó e exclui os testes do pacote instalado.
    packages=find_packages(exclude=['test']),
    # Instala o marcador ament, o manifesto e o launch de execução do percurso.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tour.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aluno',
    maintainer_email='aluno@example.com',
    description='Publica quatro metas no Nav2 e monitora a pose via odometria e TF2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Registra o executável que inicia o publicador sequencial das quatro metas.
    entry_points={
        'console_scripts': [
            'nav2_four_goals = turtlebot3_nav2_tour.nav2_four_goals:main',
        ],
    },
)