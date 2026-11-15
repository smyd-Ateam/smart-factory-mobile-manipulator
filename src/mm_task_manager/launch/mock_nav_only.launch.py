"""mock 시나리오 실행 (Gazebo · Nav2 없음).

ros2 launch mm_task_manager mock_nav_only.launch.py scenario:=s1_normal out_dir:=<결과 폴더>
S12(goal 모드)는 mission:=<share>/config/mock/mission_goal_mode.yaml 을 같이 준다.
S16~S22(정렬 · 파지까지)는 config/mock/full.yaml(nav_only false)을 더하고
mock_align_server, mock_grasp_server, mock_target_pose를 같이 띄운다 (test_mock_scenarios.py와 같음).
결과: <out_dir>/<scenario>.jsonl (상태머신 결과),
      <out_dir>/<scenario>.csv (/cmd_vel, /task_manager/state 기록),
      <out_dir>/<scenario>_goals.csv (mock_nav_server가 받은 이동 목표),
      S16~S22는 <out_dir>/<scenario>_align_goals.csv, <scenario>_grasp_goals.csv
      (mock이 받은 Align/Grasp 목표)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'mm_task_manager'

# 정렬 · 파지까지 하는 시나리오 (test/launch/test_mock_scenarios.py의 FULL과 같게 유지)
FULL_SCENARIOS = ('s16_full_success', 's17_ik_retry_success', 's18_ik_retry_exhausted',
                  's19_align_not_found', 's20_align_timeout', 's21_no_target_pose',
                  's22_grasp_miss')


def _setup(context):
    share = get_package_share_directory(PKG)
    scenario = LaunchConfiguration('scenario').perform(context)
    out_dir = os.path.expanduser(LaunchConfiguration('out_dir').perform(context))
    mission = LaunchConfiguration('mission').perform(context) or os.path.join(
        share, 'config', 'mission_test_room.yaml')
    full = scenario in FULL_SCENARIOS
    files = [mission, os.path.join(share, 'config', 'mock', 'common.yaml')]
    if full:
        files.append(os.path.join(share, 'config', 'mock', 'full.yaml'))
    files.append(os.path.join(share, 'config', 'mock', scenario + '.yaml'))
    result_log = os.path.join(out_dir, scenario + '.jsonl')

    def node(executable, extra=None, use_sim_time=True):
        params = files + [{'use_sim_time': use_sim_time}] + ([extra] if extra else [])
        return Node(package=PKG, executable=executable, name=executable,
                    parameters=params, output='screen')

    full_mocks = [
        node('mock_align_server',
             {'goal_log_path': os.path.join(out_dir, scenario + '_align_goals.csv')}),
        node('mock_grasp_server',
             {'goal_log_path': os.path.join(out_dir, scenario + '_grasp_goals.csv')}),
        node('mock_target_pose'),
    ] if full else []
    return full_mocks + [
        node('mock_clock', use_sim_time=False),
        node('mock_lifecycle'),
        node('mock_tf'),
        node('mock_nav_server', {'goal_log_path': os.path.join(out_dir, scenario + '_goals.csv')}),
        node('mock_cmd_publisher'),
        node('cmd_recorder', {'csv_path': os.path.join(out_dir, scenario + '.csv')}),
        # recorder가 IDLE부터 받도록 task_manager는 3 s(wall) 뒤에 시작 (test_mock_scenarios.py와 같은 값).
        # S16~S22는 mock이 3개 더 떠서 discovery가 늦으므로 4 s
        TimerAction(period=4.0 if full else 3.0,
                    actions=[node('task_manager', {'result_log_path': result_log})]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='s1_normal',
                              description='config/mock/<scenario>.yaml'),
        DeclareLaunchArgument('out_dir', default_value='~/.ros/mm_task_manager/mock'),
        DeclareLaunchArgument('mission', default_value='',
                              description='기본 미션 파일 (비우면 config/mission_test_room.yaml)'),
        OpaqueFunction(function=_setup),
    ])
