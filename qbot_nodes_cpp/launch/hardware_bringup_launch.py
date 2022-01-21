#
# Alpha hardware bringup launch file
#
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qbot_nodes_cpp',
            executable='base_controller',
            name='base_controller'
        ),
        Node(
            package='qbot_nodes_cpp',
            executable='local_planner',
            name='local_planner'
        ),
        Node(
            package='qbot_node_cpp',
            executable='odometry',
            name='odometry'
        )
    ])