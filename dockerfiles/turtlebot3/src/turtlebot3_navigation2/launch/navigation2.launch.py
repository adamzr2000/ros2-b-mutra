import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='')
    # Spawn pose — used to publish the ground-truth map->odom transform (below).
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Map file is selectable per world via the NAV2_MAP env var (a filename in
    # the package's map/ dir). Defaults to map.yaml (turtlebot3_world) so
    # existing setups are unchanged; start.sh sets NAV2_MAP=swarm_arena.yaml
    # when launching the walled-arena world.
    map_file_name = os.environ.get('NAV2_MAP', 'map.yaml')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            map_file_name))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace — isolates all Nav2 nodes and topics per robot'),

        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to Nav2 param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'),

        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Robot spawn X (for the ground-truth map->odom transform)'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Robot spawn Y (for the ground-truth map->odom transform)'),

        # Ground-truth localization: publish a STATIC IDENTITY map->odom transform.
        # Gazebo's diff-drive plugin reports odom in the WORLD frame (its odom frame
        # sits at the world origin, so odom->base_footprint already carries the
        # robot's full world pose) and is noise-free. Hence map == odom == world and
        # the correct map->odom is the IDENTITY — odom->base(gazebo) then gives
        # exact, drift-free localization. (A spawn-pose offset here would DOUBLE the
        # pose: map->base = spawn + odom->base = 2*spawn.) This replaces AMCL
        # scan-matching, which is unreliable in the open, symmetric arena (stale
        # map->odom -> stop-and-go + recovery -> mission-time variance). AMCL still
        # runs (below) with tf_broadcast disabled, solely so it keeps publishing
        # amcl_pose for mission_agent's "localized" readiness gate.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_ground_truth',
            namespace=namespace,
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'namespace':      namespace,
                'use_namespace':  'True',
                'map':            map_dir,
                'use_sim_time':   use_sim_time,
                'params_file':    param_dir,
                'autostart':      'true',
            }.items(),
        ),
    ])
