from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_nav2_tour',
            executable='nav2_four_goals',
            name='nav2_four_goals',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_1_x': -1.0,
                'goal_1_y': -0.5,
                'goal_1_yaw': 0.0,
                'goal_2_x': -1.0,
                'goal_2_y': 1.0,
                'goal_2_yaw': 1.57,
                'goal_3_x': 1.0,
                'goal_3_y': 1.0,
                'goal_3_yaw': 3.14,
                'goal_4_x': 1.0,
                'goal_4_y': -0.5,
                'goal_4_yaw': -1.57,
                'position_tolerance': 0.25,
                'goal_timeout_sec': 180.0,
            }],
        ),
    ])