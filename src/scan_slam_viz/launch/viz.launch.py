from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    viz_config = PathJoinSubstitution(
        [FindPackageShare('scan_slam_viz'), 'config', 'plot_config.yaml']
    )

    ld = LaunchDescription()

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

    occupancy_grid_plotter = Node(
        package='scan_slam_viz',
        executable='occupancy_grid_plotter',
        name='occupancy_grid_plotter',
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

    trajectory_logger = Node(
        package='scan_slam_viz',
        executable='trajectory_logger',
        name='trajectory_logger_node',
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

    ld.add_action(odometry_plotter)
    # ld.add_action(pose_graph_plotter)
    ld.add_action(occupancy_grid_plotter)
    # ld.add_action(timer_logger)
    # ld.add_action(trajectory_logger)
    # ld.add_action(pose_covariance_analyzer)

    return ld
