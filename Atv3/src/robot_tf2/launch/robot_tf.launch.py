from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    include_footprint_arg = DeclareLaunchArgument(
        'include_footprint',
        default_value='true',
        description='Publica base_link -> base_footprint (use false junto com TurtleBot3 Gazebo)'
    )
    include_footprint = LaunchConfiguration('include_footprint')

    return LaunchDescription([
        include_footprint_arg,

        # base_footprint: [0, 0, -0.16] m
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint',
            condition=IfCondition(include_footprint),
            arguments=['0.0', '0.0', '-0.16', '0.0', '0.0', '0.0', 'base_link', 'base_footprint']
        ),

        # lidar_link: [0, 0, +0.20] m
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar',
            arguments=['0.0', '0.0', '0.20', '0.0', '0.0', '0.0', 'base_link', 'lidar_link']
        ),

        # Roda dianteira esquerda: [+0.18, +0.14, -0.03] m
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_wheel_fl',
            arguments=['0.18', '0.14', '-0.03', '0.0', '0.0', '0.0', 'base_link', 'wheel_fl_link']
        ),

        # Roda dianteira direita: [+0.18, -0.14, -0.03] m
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_wheel_fr',
            arguments=['0.18', '-0.14', '-0.03', '0.0', '0.0', '0.0', 'base_link', 'wheel_fr_link']
        ),

        # Roda traseira esquerda: [-0.18, +0.14, -0.03] m
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_wheel_rl',
            arguments=['-0.18', '0.14', '-0.03', '0.0', '0.0', '0.0', 'base_link', 'wheel_rl_link']
        ),

        # Roda traseira direita: [-0.18, -0.14, -0.03] m
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_wheel_rr',
            arguments=['-0.18', '-0.14', '-0.03', '0.0', '0.0', '0.0', 'base_link', 'wheel_rr_link']
        ),
    ])