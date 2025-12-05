#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        
        # Include your navigator launch (which includes RViz + navigator node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("autonomy_repo"),
                    "launch",
                    "navigator.launch.py"
                ])
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items()
        ),
        
        # Frontier explorer
        Node(
            package="autonomy_repo",
            executable="frontier_explorer.py",
            name="frontier_explorer",
            output="screen"
        )
    ])
