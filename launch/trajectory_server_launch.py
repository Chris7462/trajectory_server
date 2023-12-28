from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    trajectory_server_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='trajectory_server_node',
        namespace='',
        parameters=[{
            'target_frame_name': 'map',
            'source_frame_name': 'base_link',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0
        }]
    )

    return LaunchDescription([
        trajectory_server_node
    ])
