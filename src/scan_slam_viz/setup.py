from setuptools import find_packages, setup

package_name = 'scan_slam_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/viz.launch.py']),
        ('share/' + package_name + '/config', ['config/plot_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwade19',
    maintainer_email='note2jwade@gmail.com',
    description='Visualization, analysis, and logging nodes for scan_slam',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "odometry_plotter = scan_slam_viz.odometry_plotter:main",
            "pose_graph_plotter = scan_slam_viz.pose_graph_plotter:main",
            "occupancy_grid_plotter = scan_slam_viz.occupancy_grid_plotter:main",
            "pose_covariance_analyzer = scan_slam_viz.pose_covariance_analyzer:main",
            "trajectory_logger = scan_slam_viz.trajectory_logger_node:main",
            "timer_logger = scan_slam_viz.timer_logger_node:main",
        ],
    },
)
