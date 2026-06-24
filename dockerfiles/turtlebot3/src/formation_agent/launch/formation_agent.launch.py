# Launch a single namespaced formation_agent.
#
# Typically included by the per-robot bring-up with namespace:=robotN so the
# relative interfaces (cmd_vel, formation_status) resolve under /robotN, while
# neighbor odometry and the shared goal/excluded topics are read on absolute
# names (/robotX/odom, /formation/goal, /formation/excluded). Can also be run
# standalone for testing:
#
#   ros2 launch formation_agent formation_agent.launch.py namespace:=robot1

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    tick_rate_hz = LaunchConfiguration('tick_rate_hz', default='10.0')
    formation_scale = LaunchConfiguration('formation_scale', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace, e.g. robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('tick_rate_hz', default_value='10.0',
                              description='Control / heartbeat rate (Hz)'),
        DeclareLaunchArgument('formation_scale', default_value='1.0',
                              description='Formation half-diagonal d (m)'),

        Node(
            package='formation_agent',
            executable='formation_agent',
            name='formation_agent',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'tick_rate_hz': tick_rate_hz,
                'formation_scale': formation_scale,
            }],
        ),
    ])
