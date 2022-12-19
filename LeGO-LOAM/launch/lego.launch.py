import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():



    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 -1.570795 -1.570795 0.0 camera velodyne'.split(' '),
            output='screen'
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 1.570795 0.0 1.570795 map camera'.split(' '),
            output='screen'
            ),
        Node(
            package='lego_loam',
            executable='image_projection',
            name='image_projection',
            output='screen'
        ),
        Node(
            package='lego_loam',
            executable='feature_association',
            name='feature_association',
            output='screen'
        )
    ])
