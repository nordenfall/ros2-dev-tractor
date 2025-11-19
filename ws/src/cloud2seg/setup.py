from setuptools import setup

package_name = 'cloud2seg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/project_bag.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='LiDAR->camera projection over a bag sequence',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pcd_projector_node = cloud2seg.pcd_projector_node:main',
        ],
    },
)
