from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    out_dir = LaunchConfiguration('out_dir')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'out_dir',
            default_value='~/ws/data/rellis/out'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        Node(
            package='cloud2seg',
            executable='pcd_projector_node',
            name='pcd_projector',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'out_dir': out_dir,
                'use_sim_time': use_sim_time,
                'image_topic': '/pylon_camera_node/image_raw',
                'caminfo_topic': '/pylon_camera_node/camera_info',
                'cloud_topic': '/os1_cloud_node/points',
                'camera_frame': 'pylon_camera',
                'lidar_frame': 'ouster1/os1_lidar',
                'thick': 3,
                'zmin': 0.05,
                'zmax': 200.0,
                'rmax': 0.0,
                'publish_overlay': False,
            }]
        )
    ])
