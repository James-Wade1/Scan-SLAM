from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction

def generate_launch_description():

    slam_config = PathJoinSubstitution(
        [FindPackageShare('scan_slam'), 'config', 'slam_config.yaml']
    )

    sim_config = PathJoinSubstitution(
        [FindPackageShare('scan_slam_sim'), 'config', 'sim_config.yaml']
    )

    viz_config = PathJoinSubstitution(
        [FindPackageShare('scan_slam_viz'), 'config', 'plot_config.yaml']
    )

    ld = LaunchDescription()

    scan_slam_node = Node(
        package='scan_slam',
        executable='frontend_node',
        name='slam_frontend_node',
        output='screen',
        parameters=[slam_config],
    )

    slam_simulation = Node(
        package='scan_slam_sim',
        executable='robot_sim',
        name='slam_simulation_node',
        output='screen',
        parameters=[sim_config],
    )

    odometry_plotter = Node(
        package='scan_slam_viz',
        executable='odometry_plotter',
        name='odometry_plotter_node',
        output='screen',
        parameters=[viz_config],
    )

    pose_graph_plotter = Node(
        package='scan_slam_viz',
        executable='pose_graph_plotter',
        name='pose_graph_plotter_node',
        output='screen',
        parameters=[viz_config],
    )

    odometry_publisher = Node(
        package='scan_slam_sim',
        executable='odom_publisher',
        name='odom_publisher_node',
        output='screen',
        parameters=[sim_config],
    )

    occupancy_grid_plotter = Node(
        package='scan_slam_viz',
        executable='occupancy_grid_plotter',
        name='occupancy_grid_plotter',
        output='screen',
        parameters=[viz_config],
    )

    pose_covariance_analyzer = Node(
        package='scan_slam_viz',
        executable='pose_covariance_analyzer',
        name='pose_covariance_analyzer_node',
        output='screen',
        parameters=[viz_config],
    )

    trajectory_logger = Node(
        package='scan_slam_viz',
        executable='trajectory_logger',
        name='trajectory_logger_node',
        output='screen',
        parameters=[viz_config],
    )

    timer_logger = Node(
        package='scan_slam_viz',
        executable='timer_logger',
        name='timer_logger_node',
        output='screen',
        parameters=[viz_config],
    )

    delayed_cmd_vel_pub = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='scan_slam_sim',
                executable='cmd_vel_pub',
                name='cmd_vel_pub_node',
                output='screen',
                parameters=[sim_config],
            )
        ]
    )

    ld.add_action(scan_slam_node)
    ld.add_action(slam_simulation)
    ld.add_action(odometry_plotter)
    ld.add_action(odometry_publisher)
    ld.add_action(trajectory_logger)
    ld.add_action(timer_logger)
    ld.add_action(pose_graph_plotter)
    ld.add_action(occupancy_grid_plotter)
    ld.add_action(pose_covariance_analyzer)
    ld.add_action(delayed_cmd_vel_pub)

    return ld
