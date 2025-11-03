from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kitti_writer',
            executable='kitti_writer_node',
            name='kitti_writer_node',
            output='screen',
            parameters=[{
                'input_topic': '/livox/lidar',       # подстрой под свой драйвер
                'output_dir': '/data/kitti/velodyne', # примонтируй том в докере
                'target_frame': 'base_link',
                'write_bin': True,
                'publish_topic': True,
                'intensity_field': 'intensity',       # если у Livox другое поле — поменяем
                'intensity_scale': 1.0,
                'max_keep_bins': 5                    # как ты и хотел: держим последние 5
            }]
        )
    ])
