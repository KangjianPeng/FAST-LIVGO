from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    use_rviz = LaunchConfiguration('rviz')
    rviz_launch_arg = DeclareLaunchArgument('rviz', default_value='True')
    ld.add_action(rviz_launch_arg)

    livo_params_file = os.path.join(get_package_share_directory('fast_livgo'),
                                    'config', 'avia.yaml')
    camera_params_file = os.path.join(get_package_share_directory('fast_livgo'),
                                    'config', 'camera_pinhole.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('fast_livgo'),
                                    'rviz_cfg', 'fast_livgo2.rviz')

    livo = Node(
        package='fast_livgo',
        executable='fastlivgo_mapping',
        name='laserMapping',
        parameters=[livo_params_file, camera_params_file],
        output="screen"
    )
    ld.add_action(livo)

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output="screen"
    )
    ld.add_action(rviz_cmd)
    return ld
    