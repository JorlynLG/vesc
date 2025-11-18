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
    
    #Start Left Motor
    left_motor = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace='left_motor',
        parameters=[LaunchConfiguration("config")],
        output='screen'
    )

    #Start Right Motor
    right_motor = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace='right_motor',
        parameters=[LaunchConfiguration("config")],
        output='screen'
    )

    #Start Diff Drive Converter
    diff_drive_controller = Node(
        package='vesc_diff_drive',
        executable='vesc_diff_drive_node',
        name='vesc_diff_drive_node',
        parameters=[LaunchConfiguration("config")],
        output='screen'           
    )

    #Start Odometry publisher
    odom_node = Node(
        package='vesc_diff_drive',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration("config")],
        output='screen'           
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="config",
            default_value=vesc_config,
            description="VESC yaml configuration file.",
            ),
        left_motor, 
        right_motor,
        diff_drive_controller,
        odom_node
    ])
