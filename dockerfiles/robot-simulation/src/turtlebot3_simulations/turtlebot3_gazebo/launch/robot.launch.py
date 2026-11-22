import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time     = LaunchConfiguration('use_sim_time',     default='true')
    x_pose           = LaunchConfiguration('x_pose',           default='0.0')
    y_pose           = LaunchConfiguration('y_pose',           default='0.0')
    namespace        = LaunchConfiguration('namespace',        default='')
    spawn_timeout    = LaunchConfiguration('spawn_timeout',    default='180')
    enable_nav2      = LaunchConfiguration('enable_nav2',      default='false')
    enable_formation = LaunchConfiguration('enable_formation', default='false')
    formation_scale  = LaunchConfiguration('formation_scale',  default='2.0')
    k_att            = LaunchConfiguration('k_att',            default='0.8')
    k_form           = LaunchConfiguration('k_form',           default='3.0')

    return LaunchDescription([
        DeclareLaunchArgument('namespace',     default_value='',      description='Robot namespace'),
        DeclareLaunchArgument('x_pose',        default_value='0.0',   description='Spawn X position'),
        DeclareLaunchArgument('y_pose',        default_value='0.0',   description='Spawn Y position'),
        DeclareLaunchArgument('use_sim_time',  default_value='true',  description='Use simulation time'),
        DeclareLaunchArgument('spawn_timeout', default_value='180',   description='Spawn service timeout (s)'),
        DeclareLaunchArgument('enable_nav2',   default_value='false', description='Launch Nav2 navigation stack'),
        DeclareLaunchArgument('enable_formation', default_value='false', description='Launch the formation_agent (formation experiment)'),
        DeclareLaunchArgument('formation_scale',  default_value='2.0',   description='Formation half-diagonal d (m)'),
        DeclareLaunchArgument('k_att',            default_value='0.8',   description='APF goal-pull gain'),
        DeclareLaunchArgument('k_form',           default_value='3.0',   description='APF formation-cohesion gain'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace':    namespace,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose':        x_pose,
                'y_pose':        y_pose,
                'namespace':     namespace,
                'spawn_timeout': spawn_timeout,
            }.items()
        ),

        # Per-robot mission agent. The 'enable_nav2' flag is the swarm-mission
        # switch (set by start.sh --nav2). The agent drives the robot directly
        # with velocity commands toward its assigned waypoints using ground-truth
        # odometry — no Nav2/AMCL stack, which the obstacle-free arena does not
        # need and which only added nondeterminism. Relative interface names
        # resolve under the robot namespace (/robotN/assigned_waypoints,
        # /robotN/mission_status, /robotN/odom, /robotN/cmd_vel). This node is
        # the attested + tampered target for the swarm impact experiments.
        Node(
            package='mission_agent',
            executable='mission_agent',
            name='mission_agent',
            namespace=namespace,
            output='screen',
            # goal_tolerance_m=0.5: a robot marks a waypoint reached 0.5 m out.
            # This keeps a rescuer ~0.36 m clear of a frozen (compromised) robot
            # parked on a shared waypoint, so the open-loop controller never has
            # to push through it — keeping mitigated runs collision-free and
            # deterministic. Negligible effect on the (4 m-spaced) path lengths.
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_tolerance_m': 0.5,
            }],
            condition=IfCondition(enable_nav2),
        ),

        # Per-robot formation agent (formation experiment, gated on
        # enable_formation). Decentralized APF over neighbor odometry — drives
        # cmd_vel directly using ground-truth odom, no Nav2. Relative interfaces
        # resolve under the robot namespace (/robotN/cmd_vel, /robotN/formation_status);
        # neighbor odometry and the shared goal/excluded topics are read on
        # absolute names (/robotX/odom, /formation/goal, /formation/excluded).
        # This node is the attested + tampered target for the formation impact
        # experiments (mutually exclusive with enable_nav2 in practice).
        Node(
            package='formation_agent',
            executable='formation_agent',
            name='formation_agent',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'formation_scale': formation_scale,
                'k_att': k_att,
                'k_form': k_form,
            }],
            condition=IfCondition(enable_formation),
        ),
    ])
