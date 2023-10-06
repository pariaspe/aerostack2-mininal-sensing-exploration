"""
explorer.py
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    """entrypoint
    """
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description="Drone namespace",
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument(
            'use_sim_time', description="Use sim time flag", default_value='false'),
        DeclareLaunchArgument(
            'frontier_min_area', description="Minimum area size to be a frontier (in pixels)",
            default_value='1'),
        DeclareLaunchArgument(
            'safety_distance', description="Safety distance to obstacles (drone size)",
            default_value='1.0'),
        DeclareLaunchArgument(
            'reached_dist_thresh', description="Threshold to consider point as reached",
            default_value='0.5'),
        Node(
            package="as2_path_planning",
            executable="explorer",
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frontier_min_area': LaunchConfiguration('frontier_min_area'),
                'safety_distance': LaunchConfiguration('safety_distance'),
                'reached_dist_thresh': LaunchConfiguration('reached_dist_thresh')
            }],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])
