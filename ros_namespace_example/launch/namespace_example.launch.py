from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    root_namespace_node = Node(
        package='ros_namespace_example',
        executable='namespace_example',
        name='namespace_example_node')

    namespace_1_node = Node(package='ros_namespace_example',
                            executable='namespace_example',
                            name='namespace_example_node',
                            namespace='namespace_1')

    namespace_2_node = Node(package='ros_namespace_example',
                            executable='namespace_example',
                            name='namespace_example_node',
                            namespace='namespace_2')

    return LaunchDescription([root_namespace_node, namespace_1_node, namespace_2_node])