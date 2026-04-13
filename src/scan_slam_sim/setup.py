from setuptools import find_packages, setup

package_name = 'scan_slam_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sim_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwade19',
    maintainer_email='note2jwade@gmail.com',
    description='Robot simulator and driver nodes for scan_slam testing',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "robot_sim = scan_slam_sim.robot_sim:main",
            "cmd_vel_pub = scan_slam_sim.cmd_vel_pub:main",
            "odom_publisher = scan_slam_sim.odom_publisher:main",
        ],
    },
)
