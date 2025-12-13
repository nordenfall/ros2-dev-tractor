from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloud2seg',
            executable='pcd_projector_hardcoded',
            name='pcd_projector_hardcoded',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ]
        )
    ])
