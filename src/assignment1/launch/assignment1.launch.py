#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    #Launch Turtlesim Node
    turtlesim = Node(package='turtlesim', executable='turtlesim_node', name='turtlesim_node', output='screen')
    cleaner = Node(package='assignment1', executable='cleaner.py', name='cleaner', output='screen')
    tracker = Node(package='assignment1', executable='tracker.py', name='tracker', output='screen')
    
    return [turtlesim, cleaner, tracker]


def generate_launch_description():
    #Launch Description Declaration
    ld = LaunchDescription()

    #Add Actions
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld