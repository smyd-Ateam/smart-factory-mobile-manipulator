"""mock 시나리오 S1~S15 자동 시험 (launch_testing).

colcon test로 실행한다 (README 참고). 시나리오마다 mock 노드와 task_manager를 띄우고,
task_manager가 스스로 끝나기를 SCENARIO_TIMEOUT_SEC까지만 기다린다 (멈춘 시나리오가 전체를 막지 않게).
끝난 뒤 결과 기록(JSONL), /cmd_vel · /task_manager/state 기록(CSV),
mock_nav_server가 받은 이동 목표(CSV)를 기대값과 비교한다.
"""

import csv
import json
import math
import os
import shutil
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

PKG = 'mm_task_manager'
OUT_ROOT = '~/.ros/mm_task_manager/launch_test'
SCENARIO_TIMEOUT_SEC = 45.0
START_DELAY_SEC = 2.0          # recorder가 IDLE부터 받도록 task_manager를 늦게 시작
FINAL_ZERO_SEC = 1.0           # mission_test_room.yaml final_zero_sec

OK = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'VERIFY_ARRIVAL', 'SUCCEEDED']
NAV_FAIL = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'FAILED']
ARRIVAL_FAIL = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'VERIFY_ARRIVAL', 'FAILED']
TIMEOUT_FAIL = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'CANCELING', 'FAILED']
NOT_READY = ['IDLE', 'WAIT_READY', 'FAILED']

# 시나리오: (최종 상태, error_code, detail, aborted, cancel_no_response, 단계 순서)
EXPECTED = {
    's1_normal': ('SUCCEEDED', '', '', False, False, OK),
    's2_fake_success': ('FAILED', 'NAV_FAILED', 'arrival_check', False, False, ARRIVAL_FAIL),
    's3_tf_stale': ('FAILED', 'NAV_FAILED', 'tf_stale', False, False, ARRIVAL_FAIL),
    's4_not_ready': ('FAILED', 'NAV_NOT_READY', 'lifecycle_inactive', False, False, NOT_READY),
    's5_timeout': ('FAILED', 'NAV_FAILED', 'timeout', False, False, TIMEOUT_FAIL),
    's6_cancel_no_response': ('FAILED', 'NAV_FAILED', 'timeout', True, True, TIMEOUT_FAIL),
    's7_cmd_stop': ('SUCCEEDED', '', '', False, False, OK),
    's8a_reject': ('FAILED', 'NAV_FAILED', 'rejected', False, False, NAV_FAIL),
    's8b_aborted': ('FAILED', 'NAV_FAILED', 'aborted', False, False, NAV_FAIL),
    's9_clock_stop': ('FAILED', 'UNKNOWN', 'clock_stopped', True, False, NAV_FAIL),
    's10_cmd_vel_conflict': ('FAILED', 'NAV_NOT_READY', 'cmd_vel_conflict', False, False,
                             NOT_READY),
    's11_final_stop': ('SUCCEEDED', '', '', False, False, OK),
    's12_goal_mode': ('SUCCEEDED', '', '', False, False, OK),
    's13_object_diagonal': ('SUCCEEDED', '', '', False, False, OK),
    's14_object_behind': ('SUCCEEDED', '', '', False, False, OK),
    's15_object_close': ('SUCCEEDED', '', '', False, False, OK),
}

# 기본 미션 파일 (config/ 기준). 없으면 mission_test_room.yaml (object 모드, 물체 (1.40, 0))
MISSION_BASE = {'s12_goal_mode': os.path.join('mock', 'mission_goal_mode.yaml')}

# mock_nav_server가 받아야 하는 이동 목표 (x, y, yaw). mock 차체 출발 위치는 common.yaml.
# object 모드: 물체에서 0.40 m (= standoff_list[0] 0.25 + margin 0.15), 출발 위치 → 물체 방향
_D = 0.40
_GOAL_DEFAULT = (1.0, 0.0, 0.0)    # 출발 (1.0, 0) 또는 (0, 0)에서 물체 (1.40, 0)
_GOAL_GOAL_MODE = (1.0, 0.0, 0.0)  # S12: mission_goal_mode.yaml


def _approach(obj, start):
    yaw = math.atan2(obj[1] - start[1], obj[0] - start[0])
    return (obj[0] - _D * math.cos(yaw), obj[1] - _D * math.sin(yaw), yaw)


NAV_GOAL = {name: _GOAL_DEFAULT for name in EXPECTED
            if name not in ('s4_not_ready', 's10_cmd_vel_conflict')}
NAV_GOAL.update({
    's12_goal_mode': _GOAL_GOAL_MODE,
    's4_not_ready': None,                                     # 목표 안 보냄
    's10_cmd_vel_conflict': None,
    's13_object_diagonal': _approach((2.0, 1.0), (1.0, 0.0)),   # (1.7172, 0.7172, 0.7854)
    's14_object_behind': _approach((0.0, -0.5), (1.0, 0.0)),    # (0.3578, -0.3211, -2.6779)
    's15_object_close': _approach((1.2, 0.0), (1.0, 0.0)),      # (0.80, 0, 0)
})


@pytest.mark.launch_test
@launch_testing.parametrize('scenario', list(EXPECTED))
def generate_test_description(scenario):
    share = get_package_share_directory(PKG)
    # 시나리오마다 고정 폴더에 덮어씀 (마지막 결과는 디버깅용으로 남김)
    out_dir = os.path.join(os.path.expanduser(OUT_ROOT), scenario)
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir)
    mission = MISSION_BASE.get(scenario, 'mission_test_room.yaml')
    files = [os.path.join(share, 'config', mission),
             os.path.join(share, 'config', 'mock', 'common.yaml'),
             os.path.join(share, 'config', 'mock', scenario + '.yaml')]

    def node(executable, extra=None, use_sim_time=True):
        params = files + [{'use_sim_time': use_sim_time}] + ([extra] if extra else [])
        return Node(package=PKG, executable=executable, name=executable,
                    parameters=params, output='screen')

    task_manager = node('task_manager',
                        {'result_log_path': os.path.join(out_dir, 'result.jsonl')})
    return LaunchDescription([
        node('mock_clock', use_sim_time=False),
        node('mock_lifecycle'),
        node('mock_tf'),
        node('mock_nav_server', {'goal_log_path': os.path.join(out_dir, 'goals.csv')}),
        node('mock_cmd_publisher'),
        node('cmd_recorder', {'csv_path': os.path.join(out_dir, 'record.csv')}),
        TimerAction(period=START_DELAY_SEC, actions=[task_manager]),
        launch_testing.actions.ReadyToTest(),
    ]), {'task_manager': task_manager, 'out_dir': out_dir, 'scenario': scenario}


class TestRun(unittest.TestCase):

    def test_task_manager_finishes(self, proc_info, task_manager):
        # 멈춘 시나리오도 여기서 시간 제한으로 끝난다
        proc_info.assertWaitForShutdown(process=task_manager, timeout=SCENARIO_TIMEOUT_SEC)


def _load(out_dir):
    with open(os.path.join(out_dir, 'result.jsonl')) as f:
        record = json.loads(f.read().splitlines()[-1])
    with open(os.path.join(out_dir, 'record.csv')) as f:
        rows = list(csv.DictReader(f))
    states = [(float(r['sim_time']), float(r['wall_time']), r['state'])
              for r in rows if r['topic'] == 'state']
    cmds = [(float(r['sim_time']), float(r['wall_time']), float(r['vx']), float(r['wz']))
            for r in rows if r['topic'] == 'cmd_vel']
    return record, states, cmds


@launch_testing.post_shutdown_test()
class TestResult(unittest.TestCase):

    def test_exit_code(self, proc_info, task_manager):
        launch_testing.asserts.assertExitCodes(proc_info, process=task_manager)

    def test_record(self, out_dir, scenario):
        record, _, _ = _load(out_dir)
        final, code, detail, aborted, cnr, _ = EXPECTED[scenario]
        self.assertEqual(
            (record['final_state'], record['error_code'], record['detail'],
             record['aborted'], record['cancel_no_response']),
            (final, code, detail, aborted, cnr))

    def test_state_sequence(self, out_dir, scenario):
        # /task_manager/state: 단계가 바뀔 때만, interfaces.md v0.14 이름으로
        # IDLE은 노드 시작 직후 약 0.1 s만 유지되고 depth 1 transient_local이라,
        # discovery가 늦은 구독자는 못 받을 수 있다 → 맨 앞 IDLE만 선택
        _, states, _ = _load(out_dir)
        got = [s for _, _, s in states]
        expected = EXPECTED[scenario][5]
        self.assertIn(got, (expected, expected[1:]))

    def test_cmd_vel_only_while_navigate(self, out_dir):
        # 5.0: 0이 아닌 /cmd_vel은 NAVIGATE 동안에만 (경계에서 한 주기 여유)
        _, states, cmds = _load(out_dir)
        windows = []
        for (t, _, s), nxt in zip(states, states[1:] + [(float('inf'), 0, '')]):
            if s == 'NAVIGATE':
                windows.append((t, nxt[0] + 0.15))
        bad = [(t, vx, wz) for t, _, vx, wz in cmds
               if (vx, wz) != (0.0, 0.0) and not any(a <= t <= b for a, b in windows)]
        self.assertEqual(bad, [])

    def test_final_stop(self, out_dir, scenario):
        # 끝난 뒤에는 0만, final_zero_sec 뒤 task_manager 종료 → /cmd_vel 발행 없음
        _, states, cmds = _load(out_dir)
        end_wall = states[-1][1]
        after = [(w, vx, wz) for _, w, vx, wz in cmds if w > end_wall]
        self.assertTrue(all((vx, wz) == (0.0, 0.0) for _, vx, wz in after), after[:5])
        if scenario != 's10_cmd_vel_conflict':     # S10은 mock 발행자가 계속 0을 보냄
            late = [w - end_wall for w, _, _ in after if w > end_wall + FINAL_ZERO_SEC + 0.5]
            self.assertEqual(late, [])
            self.assertGreater(len(after), 3)      # 0을 실제로 보냈는지

    def test_scenario_specific(self, out_dir, scenario):
        _, states, cmds = _load(out_dir)
        nonzero = [t for t, _, vx, wz in cmds if (vx, wz) != (0.0, 0.0)]
        if scenario in ('s4_not_ready', 's8a_reject', 's10_cmd_vel_conflict'):
            self.assertEqual(nonzero, [])
        if scenario == 's7_cmd_stop':
            # mock이 1.5 s 뒤 /nav2/cmd_vel을 끊음 → 0.5 s 안에 0 (여유 0.3 s)
            self.assertLessEqual(max(nonzero) - min(nonzero), 1.5 + 0.5 + 0.3)
        if scenario in ('s1_normal', 's2_fake_success', 's7_cmd_stop', 's11_final_stop'):
            self.assertGreater(len(nonzero), 5)

    def test_nav_goal_sent(self, out_dir, scenario):
        # mock_nav_server가 받은 목표 = 기대값 (map 프레임, 1 mm / 0.001 rad 이내), 재시도 없음
        with open(os.path.join(out_dir, 'goals.csv')) as f:
            goals = list(csv.DictReader(f))
        expected = NAV_GOAL[scenario]
        if expected is None:
            self.assertEqual(goals, [])
            return
        self.assertEqual(len(goals), 1)
        g = goals[0]
        self.assertEqual(g['frame_id'], 'map')
        self.assertAlmostEqual(float(g['x']), expected[0], delta=1e-3)
        self.assertAlmostEqual(float(g['y']), expected[1], delta=1e-3)
        dyaw = (float(g['yaw']) - expected[2] + math.pi) % (2 * math.pi) - math.pi
        self.assertAlmostEqual(dyaw, 0.0, delta=1e-3)

    def test_nav_goal_mode(self, out_dir, scenario):
        record, _, _ = _load(out_dir)
        self.assertEqual(record['nav_goal_mode'], 'goal' if scenario in MISSION_BASE else 'object')
        if scenario in ('s13_object_diagonal', 's14_object_behind', 's15_object_close'):
            # teleport로 계산한 목표에 도착 → 물체까지 0.40 m, 물체를 바라봄
            self.assertAlmostEqual(record['approach_distance'], _D, delta=1e-9)
            self.assertAlmostEqual(record['arrival']['object_dist'], _D, delta=0.005)
            self.assertLess(record['arrival']['dist'], 0.005)
