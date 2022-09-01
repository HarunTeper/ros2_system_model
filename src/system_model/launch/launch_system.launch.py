#!/usr/bin/env python3
"""ROS2 Python shebang"""

import os.path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext


def include_system_description(context: LaunchContext):
    """Returns nodes"""

    # Package Directories
    pkg_dir = get_package_share_directory('system_model')

    # Paths to folders and files
    default_config_file = os.path.join(pkg_dir, 'config', str(
        context.launch_configurations['system_config_yaml_file_name']))

    # Nodes
    system_composition_node = Node(
        package='system_model',
        executable='system_composition',
        output='screen',
        arguments=[
            '-system', default_config_file]
    )

    cmds = []
    cmds.append(system_composition_node)
    return cmds


def generate_launch_description():
    """Returns launch description"""

    # Launch Arguments
    system_config_yaml_file_name_arg = DeclareLaunchArgument(
        'system_config_yaml_file_name',
        default_value="empty.yaml",
        description='name of the yaml file in the config folder'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(system_config_yaml_file_name_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_system_description))

    return launch_description
