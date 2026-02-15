import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Declare launch arguments
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    namespace = LaunchConfiguration('namespace', default='')
    entity_name = PythonExpression(["'", namespace, "_", TURTLEBOT3_MODEL, "'"])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for the robot'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='X position for the robot'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position for the robot'),
        DeclareLaunchArgument('spawn_timeout', default_value='60', description='Timeout for spawn entity'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', entity_name,
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
                '-robot_namespace', namespace,  # Pass the namespace directly
                '-timeout', LaunchConfiguration('spawn_timeout')  # Add timeout argument            
            ],
            output='screen',
        ),
    ])
