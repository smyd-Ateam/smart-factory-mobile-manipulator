"""모의 입력으로 Align 액션 서버 시험.

모의 차체·카메라 + Align 서버(+ RViz)를 띄운다. 목표는 따로 보낸다.

  ros2 launch visual_servoing mock_align.launch.py
  ros2 launch visual_servoing mock_align.launch.py delay_sec:=0.5 noise_px:=2.0 noise_depth:=0.005

목표 한 번 보내기 (다른 터미널):
  ros2 action send_goal /servoing/align project_interfaces/action/Align \
    "{target_class: pin, standoff_m: 0.25, timeout_sec: 60.0}" --feedback

반복 시험 (다른 터미널):
  ros2 run visual_servoing mock_trials --ros-args -p trials:=20
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
        arg('target_class', 'pin', '모의 검출이 내는 클래스 이름'),
        arg('standoff_m', '0.25', 'RViz 목표 거리 원, 실제 오차 로그에 쓰는 거리 (m)'),
        arg('delay_sec', '0.0', '모의 측정 지연 (s)'),
        arg('noise_px', '0.0', '모의 픽셀 노이즈 표준편차 (px)'),
        arg('noise_depth', '0.0', '모의 깊이 노이즈 표준편차 (m)'),
        arg('rviz', 'true', 'true면 RViz로 차체·물체를 위에서 본 화면 표시'),
    ]

    mock = Node(
        package='visual_servoing', executable='mock_scene_node', output='screen',
        parameters=[{
            'target_class': LaunchConfiguration('target_class'),
            'standoff_m': f('standoff_m'),
            'delay_sec': f('delay_sec'),
            'noise_px': f('noise_px'),
            'noise_depth': f('noise_depth'),
        }],
    )
    align = Node(package='visual_servoing', executable='align_node', output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(get_package_share_directory('visual_servoing'),
                                      'rviz', 'mock_align.rviz')],
        # WSL에서 GPU로 그리면 꺼지는 경우가 있어 CPU 렌더링 (mm_description과 같음)
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription(args + [mock, align, rviz])
