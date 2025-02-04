from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='chatter',
        description='Topic name for publishing messages'
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Message publishing rate (Hz)'
    )
    message_arg = DeclareLaunchArgument(
        'message',
        default_value='Hello from dummy_publisher',
        description='Message content to publish'
    )

    # Retrieve launch arguments
    namespace = LaunchConfiguration('namespace')
    topic_name = LaunchConfiguration('topic_name')
    publish_rate = LaunchConfiguration('publish_rate')
    message = LaunchConfiguration('message')

    # Define the node with the dynamically set namespace and parameters
    node = Node(
        package='py_example_package',
        executable='dummy_publisher',
        namespace=namespace,
        name='dummy_publisher',
        output='screen',
        parameters=[
            {'topic_name': topic_name},
            {'publish_rate': publish_rate},
            {'message': message}
        ]
    )

    return LaunchDescription([
        namespace_arg,
        topic_name_arg,
        publish_rate_arg,
        message_arg,
        node
    ])
