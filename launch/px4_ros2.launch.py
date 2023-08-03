#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'),


        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'),

        # # Bridge gazebo
        # ExecuteProcess(
        #     cmd=[FindExecutable(name='ros2'),
        #         'run ros_gz_bridge parameter_bridge  --ros-args -p config_file:=YOURPATH/your_file.yaml'
        #         ],
        #     output='screen',
        #     shell=True
        # ),

        Node(
            package='px4',
            executable='./bin/px4'
            #arguments=["udp4", "-p", "8888", "-v"],
            on_exit=launch.actions.Shutdown()
        ),

        Node(
            package='px4',
            executable='MicroXRCEAgent',
            arguments=["udp4", "-p", "8888", "-v"]
        )


        # TODO:
        #  px4 bridge node for NavSat, IMU, robot description, parameters, services, etc

        # gazebo with RGBD and VIO or SLAM node

        # foxglove and rviz visualization

        # clock?

        # sitl example
        # hardware example


        # launch sitl sim
        # launch offboard simulation

        # namespaces for launch?


    ])
