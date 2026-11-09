"""상태머신만 실행. Nav2는 따로 (mm_navigation nav2.launch.py, standalone:=false).

ros2 launch mm_task_manager task_manager.launch.py mission:=<미션 YAML>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_mission = os.path.join(get_package_share_directory('mm_task_manager'),
                                   'config', 'mission_test_room.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value=default_mission),
        Node(package='mm_task_manager', executable='task_manager', name='task_manager',
             parameters=[LaunchConfiguration('mission'), {'use_sim_time': True}],
             output='screen'),
    ])
