import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
      get_package_share_directory('md25_controller'),
      'config',
      'hardware.yaml'
    )

    return LaunchDescription([
        Node(
            package='md25_controller',
            executable='motor_controller_node',
            name='motor_controller',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='md25_controller',
            executable='odometry_node',
            name='wheels_odometry',
            parameters=[config],
            output='screen'
        ),
                Node(
            package='md25_controller',
            executable='md25_base_controller_node',
            name='md25_base_controller',
            parameters=[config],
            output='screen'
        )
    ])
