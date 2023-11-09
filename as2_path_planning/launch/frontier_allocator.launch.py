"""
fontier_allocator.launch.py
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    """entrypoint
    """
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description="Namespace",
                              default_value=''),
        DeclareLaunchArgument(
            'use_sim_time', description="Use sim time flag", default_value='false'),

        Node(
            package="as2_path_planning",
            executable="frontier_allocator",
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])