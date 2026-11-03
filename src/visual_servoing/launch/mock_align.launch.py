"""모의 입력으로 베이스 정렬 시험.

  ros2 launch visual_servoing mock_align.launch.py
  ros2 launch visual_servoing mock_align.launch.py delay_sec:=0.5 noise_px:=2.0 noise_depth:=0.005
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        arg('standoff_m', '0.30', 'base_link 원점에서 물체까지 목표 거리 (m)'),
        arg('delay_sec', '0.0', '모의 측정 지연 (s)'),
        arg('noise_px', '0.0', '모의 픽셀 노이즈 표준편차 (px)'),
        arg('noise_depth', '0.0', '모의 깊이 노이즈 표준편차 (m)'),
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
    return LaunchDescription(args + [mock, align])
