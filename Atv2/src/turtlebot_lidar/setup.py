from setuptools import find_packages, setup

package_name = 'turtlebot_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
entry_points={
        'console_scripts': [
            'lidar_plotter = turtlebot_lidar.lidar_plotter:main',
            'reactive_navigation = turtlebot_lidar.reactive_navigation:main',
        ],
    },
)
