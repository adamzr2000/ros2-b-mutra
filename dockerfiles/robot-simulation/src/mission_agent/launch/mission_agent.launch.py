# Launch a single namespaced mission_agent.
#
# Typically included by the per-robot bring-up with namespace:=robotN so all
# interfaces resolve under /robotN (assigned_waypoints, mission_status,
# navigate_to_pose). Can also be run standalone for testing:
#
#   ros2 launch mission_agent mission_agent.launch.py namespace:=robot1

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    tick_rate_hz = LaunchConfiguration('tick_rate_hz', default='10.0')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace, e.g. robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('tick_rate_hz', default_value='10.0',
                              description='Mission tick / heartbeat rate (Hz)'),

        Node(
            package='mission_agent',
            executable='mission_agent',
            name='mission_agent',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'tick_rate_hz': tick_rate_hz,
            }],
        ),
    ])
