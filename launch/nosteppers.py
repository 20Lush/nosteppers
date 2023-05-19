from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nosteppers',
            namespace='nosteppers',
            executable='joint_server',
            name='joint_server'
        ),
        Node(
            package='nosteppers',
            namespace='nosteppers',
            executable='dynamixel_sync_read_write',
            name='srw_server'
        ),
        Node(
            package='nosteppers',
            namespace='nosteppers',
            executable='joint_planned_path',
            name='pp_controls'
        )
    ])