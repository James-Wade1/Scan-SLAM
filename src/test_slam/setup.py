from setuptools import find_packages, setup

package_name = 'test_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_plots.launch.py']),
        ('share/' + package_name + '/config', ['config/plot_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwade19',
    maintainer_email='note2jwade@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "robot_sim = test_slam.robot_sim:main",
            "odometry_plotter = test_slam.odometry_plotter:main",
            "cmd_vel_pub = test_slam.cmd_vel_pub:main",
            "odom_publisher = test_slam.odom_publisher:main",
            "pose_graph_plotter = test_slam.pose_graph_plotter:main",
            "occupancy_grid_plotter = test_slam.occupancy_grid_plotter:main",
            "pose_covariance_analyzer = test_slam.pose_covariance_analyzer:main",
            "trajectory_logger = test_slam.trajectory_logger_node:main",
            "timer_logger = test_slam.timer_logger_node:main",
        ],
    },
)
