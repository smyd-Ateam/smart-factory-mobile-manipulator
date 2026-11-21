"""YOLO 검출 노드 실행.

  ros2 launch mm_perception detector.launch.py model:=$HOME/yolo_models/synth_v1_640.pt
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("model", default_value=os.path.expanduser("~/yolo_models/synth_v1_640.pt")),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("conf", default_value="0.5"),
        DeclareLaunchArgument("save_dir", default_value=""),
    ]
    node = Node(
        package="mm_perception",
        executable="yolo_detector",
        parameters=[{
            "model": LaunchConfiguration("model"),
            "imgsz": LaunchConfiguration("imgsz"),
            "conf": LaunchConfiguration("conf"),
            "save_dir": LaunchConfiguration("save_dir"),
            "use_sim_time": True,
        }],
        output="screen",
    )
    return LaunchDescription(args + [node])
