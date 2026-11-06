"""시뮬 + slam_toolbox + RViz (지도 만들기).

사용 예
  Mac UTM VM : ros2 launch mm_navigation slam.launch.py software_gl:=true headless:=true rviz_gpu:=true
  WSL2       : ros2 launch mm_navigation slam.launch.py software_gl:=true

시뮬 옵션(software_gl, headless, rviz, rviz_gpu, render_engine)은 sim.launch.py로 그대로 전달된다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('mm_navigation')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mm_description'), 'launch', 'sim.launch.py')),
        launch_arguments={
            'rviz_config': os.path.join(pkg, 'rviz', 'nav.rviz'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg, 'config', 'slam_params.yaml'),
        }.items(),
    )

    return LaunchDescription([sim, slam])
