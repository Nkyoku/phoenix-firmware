from launch import LaunchDescription
from launch_ros.actions import Node
import socket

hostname = socket.gethostname()

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'phoenix',
            node_namespace = hostname,
            node_executable = 'stream',
            node_name = 'phoenix_stream'
        ),
        Node(
            package = 'phoenix',
            node_namespace = hostname,
            node_executable = 'adc3',
            node_name = 'phoenix_battery'
        ),
        Node(
            package = 'phoenix',
            node_namespace = hostname,
            node_executable = 'command',
            node_name = 'phoenix_command'
        )
    ])
