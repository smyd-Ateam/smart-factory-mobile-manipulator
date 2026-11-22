"""YOLO 검출 노드 실행.

  ros2 launch mm_perception detector.launch.py model:=$HOME/yolo_models/synth_v1_640.pt
  ros2 launch mm_perception detector.launch.py model:=... image_reliability:=reliable image_depth:=2
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
        DeclareLaunchArgument("image_reliability", default_value="best_effort",
                              description="이미지 구독 QoS: best_effort 또는 reliable"),
        DeclareLaunchArgument("image_depth", default_value="5"),
        DeclareLaunchArgument("min_box_fill", default_value="0.10",
                              description="박스 안에서 배경과 다른 픽셀 비율이 이보다 작으면 버림 (0이면 끔)"),
    ]
    node = Node(
        package="mm_perception",
        executable="yolo_detector",
        parameters=[{
            "model": LaunchConfiguration("model"),
            "imgsz": LaunchConfiguration("imgsz"),
            "conf": LaunchConfiguration("conf"),
            "save_dir": LaunchConfiguration("save_dir"),
            "image_reliability": LaunchConfiguration("image_reliability"),
            "image_depth": LaunchConfiguration("image_depth"),
            "min_box_fill": LaunchConfiguration("min_box_fill"),
            "use_sim_time": True,
        }],
        output="screen",
    )
    return LaunchDescription(args + [node])
