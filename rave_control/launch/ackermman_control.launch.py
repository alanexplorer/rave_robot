#!/usr/bin/env python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    ackermann_steering_controller = os.path.join(
        get_package_share_directory('rave_control'),
        'config',
        'ctrl_ackermann_steering_controller.yaml'
        )

    gains = os.path.join(
        get_package_share_directory('rave_control'),
        'config',
        'ctrl_gains.yaml'
        )    

    joint_state_publisher = os.path.join(
        get_package_share_directory('rave_control'),
        'config',
        'ctrl_joint_state_publisher.yaml'
        )    

    urdf = os.path.join(
        get_package_share_directory('rave_description'),
        'urdf',
        'rave.urdf.xacro'
    )

    xacro_file = xacro.process_file(urdf)
    robot_description = {'robot_description': xacro_file.toxml() }


    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            # parameters=[
            #     robot_description, 
            #     ackermann_steering_controller, 
            #     gains, 
            #     joint_state_publisher],
            parameters=[
                ackermann_steering_controller, 
                joint_state_publisher],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
                },
            )
    ])
