import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("fast_livgo")
    default_output = os.path.expanduser("~/.ros/fast_livgo_rtk/great_campus01")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("output_dir", default_value=default_output),
        Node(
            package="fast_livgo",
            executable="fastlivgo_mapping",
            name="laserMapping",
            output="screen",
            parameters=[
                os.path.join(share_dir, "config", "GREAT_campus01.yaml"),
                os.path.join(share_dir, "config", "camera_GREAT_20201027.yaml"),
                {"output.directory": LaunchConfiguration("output_dir")},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", os.path.join(share_dir, "rviz_cfg", "fast_livgo2.rviz")],
            output="screen",
        ),
    ])
