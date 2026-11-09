"""모의 입력으로 베이스 정렬 시험.

  ros2 launch visual_servoing mock_align.launch.py
  ros2 launch visual_servoing mock_align.launch.py delay_sec:=0.5 noise_px:=2.0 noise_depth:=0.005
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    def f(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    args = [
        arg('target_class', 'pin', '정렬할 클래스 이름'),
        arg('standoff_m', '0.25', 'base_link 원점에서 물체까지 목표 거리 (m). 초기 시험값 0.25 (interfaces.md, B 제안)'),
        arg('delay_sec', '0.0', '모의 측정 지연 (s)'),
        arg('noise_px', '0.0', '모의 픽셀 노이즈 표준편차 (px)'),
        arg('noise_depth', '0.0', '모의 깊이 노이즈 표준편차 (m)'),
        arg('rviz', 'true', 'true면 RViz로 차체·물체를 위에서 본 화면 표시'),
    ]

    target = LaunchConfiguration('target_class')

    mock = Node(
        package='visual_servoing', executable='mock_scene_node', output='screen',
        parameters=[{
            'target_class': target,
            'standoff_m': f('standoff_m'),
            'delay_sec': f('delay_sec'),
            'noise_px': f('noise_px'),
            'noise_depth': f('noise_depth'),
        }],
    )
    align = Node(
        package='visual_servoing', executable='align_node', output='screen',
        parameters=[{
            'target_class': target,
            'standoff_m': f('standoff_m'),
            # 모의 지연이 있어도 측정값이 오래됐다고 멈추지 않도록 여유
            'max_measurement_age': 1.5,
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(get_package_share_directory('visual_servoing'),
                                      'rviz', 'mock_align.rviz')],
        # WSL에서 GPU로 그리면 꺼지는 경우가 있어 CPU 렌더링 (mm_description과 같음)
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription(args + [mock, align, rviz])
