from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='px4',
            executable='MicroXRCEAgent',
            name='MicroXRCEAgent',
            arguments=["udp4", "-p", "8888", "-v6"]
        )


    ])
