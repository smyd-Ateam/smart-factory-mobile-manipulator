"""mock 시나리오 실행 (Gazebo · Nav2 없음, nav_only).

ros2 launch mm_task_manager mock_nav_only.launch.py scenario:=s1_normal out_dir:=<결과 폴더>
결과: <out_dir>/<scenario>.jsonl (상태머신 결과),
      <out_dir>/<scenario>.csv (/cmd_vel, /task_manager/state 기록)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'mm_task_manager'


def _setup(context):
    share = get_package_share_directory(PKG)
    scenario = LaunchConfiguration('scenario').perform(context)
    out_dir = os.path.expanduser(LaunchConfiguration('out_dir').perform(context))
    files = [os.path.join(share, 'config', 'mission_test_room.yaml'),
             os.path.join(share, 'config', 'mock', 'common.yaml'),
             os.path.join(share, 'config', 'mock', scenario + '.yaml')]
    result_log = os.path.join(out_dir, scenario + '.jsonl')

    def node(executable, extra=None, use_sim_time=True):
        params = files + [{'use_sim_time': use_sim_time}] + ([extra] if extra else [])
        return Node(package=PKG, executable=executable, name=executable,
                    parameters=params, output='screen')

    return [
        node('mock_clock', use_sim_time=False),
        node('mock_lifecycle'),
        node('mock_tf'),
        node('mock_nav_server'),
        node('mock_cmd_publisher'),
        node('cmd_recorder', {'csv_path': os.path.join(out_dir, scenario + '.csv')}),
        # recorder가 IDLE부터 받도록 task_manager는 2 s(wall) 뒤에 시작
        TimerAction(period=2.0, actions=[node('task_manager', {'result_log_path': result_log})]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='s1_normal',
                              description='config/mock/<scenario>.yaml'),
        DeclareLaunchArgument('out_dir', default_value='~/.ros/mm_task_manager/mock'),
        OpaqueFunction(function=_setup),
    ])
