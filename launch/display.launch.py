# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 YaolinDong
"""Display the standalone H7C model; no hardware drivers are launched."""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('infrabot_description'))
    gui = LaunchConfiguration('gui')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show joint sliders'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (share/'urdf/infrabot_h7c.urdf').read_text(),
                          'use_sim_time': sim_time}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'use_sim_time': sim_time}], condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': sim_time}], condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share/'rviz/infrabot.rviz')],
             parameters=[{'use_sim_time': sim_time}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
