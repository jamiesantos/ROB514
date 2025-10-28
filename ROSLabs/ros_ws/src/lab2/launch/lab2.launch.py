from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition

from pathlib import Path


def generate_launch_description():
    ld = LaunchDescription()

    declared_arguments = []
    # Changet the sorld
    declared_arguments.append(DeclareLaunchArgument("world_map", default_value="simple_world_launch.xml", 
                                  description="One of the .xml files in rob_stage/launch"))
    declared_arguments.append(DeclareLaunchArgument('launch_rviz', default_value="true", 
                                  description="Launch RViz for visualization."))

    stage_node = Node(
        package="$(find rob_stage)/launch/",
        executable=LaunchConfiguration("world_map"),
        name="stage"
    )

    rviz_config_file = PathJoinSubstitution(
        ["$(find lab2)", "config", "driver.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        executable="rviz2",
        name="rviz2_driver",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {
                "use_sim_time": "true",
            },
        ],
    )

    send_points_node = Node(
        package="lab2",
        executable="send_points"
    )

    ld.add_action(stage_node)
    ld.add_action(rviz_node)
    ld.add_action(send_points_node)

    return ld
