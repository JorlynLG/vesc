import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    vesc_config = os.path.join(
        get_package_share_directory('vesc_diff_drive'),
        'params',
        'vesc_config.yaml'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            name="config",
            default_value=vesc_config,
            description="VESC yaml configuration file.",
            ),

        #Start Left Motor
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='left_vesc_driver_node',
            namespace='left_motor',
            parameters=[LaunchConfiguration("config")],
            output='screen'
        ),

        # #Start Right Motor
        # Node(
        #     package='vesc_driver',
        #     executable='vesc_driver_node',
        #     name='right_vesc_driver_node',
        #     namespace='right_motor',
        #     parameters=[LaunchConfiguration("config")],
        #     output='screen'
        # ),
    ])