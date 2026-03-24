from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create node
    node = Node(
        package='engineai_ros2_node_template',
        executable='engineai_ros2_node_template',
        name='engineai_ros2_node_template',
        output='screen',
    )

    return LaunchDescription([
        node
    ])
