from setuptools import find_packages, setup

package_name = 'ros_sri'

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
            'pose_subscriber = ros_sri.pose_subscriber:main',
            'square_publisher = ros_sri.square_publisher:main',
            'position_controller = ros_sri.position_controller:main',
        ],
    },
)
