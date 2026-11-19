"""mock 시나리오 S1~S22 자동 시험 (launch_testing).

colcon test로 실행한다 (README 참고). 시나리오마다 mock 노드와 task_manager를 띄우고,
task_manager가 스스로 끝나기를 SCENARIO_TIMEOUT_SEC까지만 기다린다 (멈춘 시나리오가 전체를 막지 않게).
끝난 뒤 결과 기록(JSONL), /cmd_vel · /task_manager/state 기록(CSV),
mock_nav_server가 받은 이동 목표(CSV)를 기대값과 비교한다.
S16~S23(정렬 · 파지까지, nav_only false)는 mock Align/Grasp 서버와 mock target_pose를 같이 띄우고
받은 Align/Grasp 목표(CSV)도 비교한다. 정렬 중 Near→Pre-grasp 전환 요청(결과 기록 ·
task_manager "not sent" 경고 로그)도 확인한다 (기준 0.35 m, 희선님 확정 PR #1 2026-11-17).
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
# recorder가 먼저 뜨도록 task_manager를 늦게 시작. WAIT_READY부터 받는 것은 이 지연이 아니라
# wait_state_subscribers(common.yaml)가 맡는다 (새로 뜬 task_manager의 discovery가 2~3 s 걸릴 수 있음)
START_DELAY_SEC = 3.0
FULL_START_DELAY_SEC = 4.0     # S16~S23은 mock이 3개 더 떠서 discovery가 늦음 (2.0 s면 WAIT_READY까지 놓친 적 있음)
FINAL_ZERO_SEC = 1.0           # mission_test_room.yaml final_zero_sec

OK = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'VERIFY_ARRIVAL', 'SUCCEEDED']
NAV_FAIL = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'FAILED']
ARRIVAL_FAIL = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'VERIFY_ARRIVAL', 'FAILED']
TIMEOUT_FAIL = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'CANCELING', 'FAILED']
NOT_READY = ['IDLE', 'WAIT_READY', 'FAILED']
_TO_ALIGN = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'VERIFY_ARRIVAL', 'ALIGN']
FULL_OK = _TO_ALIGN + ['GRASP', 'SUCCEEDED']
REALIGN_OK = _TO_ALIGN + ['GRASP', 'ALIGN', 'GRASP', 'SUCCEEDED']
REALIGN_FAIL = _TO_ALIGN + ['GRASP', 'ALIGN', 'GRASP', 'ALIGN', 'GRASP', 'FAILED']
ALIGN_FAIL = _TO_ALIGN + ['FAILED']
ALIGN_CANCEL = _TO_ALIGN + ['CANCELING', 'FAILED']
GRASP_FAIL = _TO_ALIGN + ['GRASP', 'FAILED']

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
    # 정렬 · 파지까지 (nav_only false)
    's16_full_success': ('SUCCEEDED', '', '', False, False, FULL_OK),
    's17_ik_retry_success': ('SUCCEEDED', '', '', False, False, REALIGN_OK),
    's18_ik_retry_exhausted': ('FAILED', 'IK_FAILED', '', False, False, REALIGN_FAIL),
    's19_align_not_found': ('FAILED', 'OBJECT_NOT_FOUND', '', False, False, ALIGN_FAIL),
    's20_align_timeout': ('FAILED', 'ALIGN_TIMEOUT', 'timeout', False, False, ALIGN_CANCEL),
    's21_no_target_pose': ('FAILED', 'OBJECT_NOT_FOUND', 'target_pose_stale', False, False,
                           GRASP_FAIL),
    's22_grasp_miss': ('FAILED', 'GRASP_MISS', '', False, False, GRASP_FAIL),
    's23_target_far': ('SUCCEEDED', '', '', False, False, FULL_OK),
}

# 정렬 · 파지 시나리오: (Align 호출마다 standoff_m, Grasp 호출 수, 실패 단계, 재정렬 횟수)
# standoff_list [0.25, 0.22, 0.28] (mission_test_room.yaml)
FULL = {
    's16_full_success': ([0.25], 1, '', 0),
    's17_ik_retry_success': ([0.25, 0.22], 2, '', 1),
    's18_ik_retry_exhausted': ([0.25, 0.22, 0.28], 3, 'GRASP', 2),
    's19_align_not_found': ([0.25], 0, 'ALIGN', 0),
    's20_align_timeout': ([0.25], 0, 'ALIGN', 0),
    's21_no_target_pose': ([0.25], 0, 'GRASP', 0),
    's22_grasp_miss': ([0.25], 1, 'GRASP', 0),
    's23_target_far': ([0.25], 1, '', 0),
}

# mock_target_pose가 발행하는 물체 x (base_link). full.yaml 0.25, S23만 0.40 (전환 기준보다 멂)
TARGET_X = {name: 0.25 for name in FULL}
TARGET_X['s23_target_far'] = 0.40

# 정렬 중 Near→Pre-grasp 전환을 요청한 거리 (결과 기록 pregrasp_request_distance_m).
# None = 요청 없음 → task_manager "not sent" 경고도 없어야 함, 값이 있으면 경고 정확히 한 번
# (재정렬이 있어도 한 실행에 한 번). S21은 target_pose가 없어서 요청 없음
PREGRASP_DISTANCE = {name: None for name in EXPECTED}
PREGRASP_DISTANCE.update({name: 0.25 for name in FULL
                          if name not in ('s21_no_target_pose', 's23_target_far')})
NOT_SENT_LOG = 'not sent: request interface not decided yet'
SERVO_VX = 0.03   # mock_align_server가 정렬 중 /servoing/cmd_vel로 보내는 값 (full.yaml)

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
             os.path.join(share, 'config', 'mock', 'common.yaml')]
    if scenario in FULL:
        files.append(os.path.join(share, 'config', 'mock', 'full.yaml'))
    files.append(os.path.join(share, 'config', 'mock', scenario + '.yaml'))

    def node(executable, extra=None, use_sim_time=True):
        params = files + [{'use_sim_time': use_sim_time}] + ([extra] if extra else [])
        return Node(package=PKG, executable=executable, name=executable,
                    parameters=params, output='screen')

    task_manager = node('task_manager',
                        {'result_log_path': os.path.join(out_dir, 'result.jsonl')})
    full_mocks = [
        node('mock_align_server', {'goal_log_path': os.path.join(out_dir, 'align_goals.csv')}),
        node('mock_grasp_server', {'goal_log_path': os.path.join(out_dir, 'grasp_goals.csv')}),
        node('mock_target_pose'),
    ] if scenario in FULL else []
    return LaunchDescription(full_mocks + [
        node('mock_clock', use_sim_time=False),
        node('mock_lifecycle'),
        node('mock_tf'),
        node('mock_nav_server', {'goal_log_path': os.path.join(out_dir, 'goals.csv')}),
        node('mock_cmd_publisher'),
        node('cmd_recorder', {'csv_path': os.path.join(out_dir, 'record.csv')}),
        TimerAction(period=FULL_START_DELAY_SEC if scenario in FULL else START_DELAY_SEC,
                    actions=[task_manager]),
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
        # /task_manager/state: 단계가 바뀔 때만, interfaces.md v0.14 이름으로, IDLE부터 빠짐없이.
        # transient_local depth 17(state_qos.py)이라 recorder가 늦게 붙어도 지난 단계를 받는다
        _, states, _ = _load(out_dir)
        got = [s for _, _, s in states]
        self.assertEqual(got, EXPECTED[scenario][5])

    def test_cmd_vel_only_while_navigate(self, out_dir):
        # 5.0: 0이 아닌 /cmd_vel은 NAVIGATE(Nav2)와 ALIGN(정렬) 동안에만 (경계에서 한 주기 여유)
        _, states, cmds = _load(out_dir)
        windows = []
        for (t, _, s), nxt in zip(states, states[1:] + [(float('inf'), 0, '')]):
            if s in ('NAVIGATE', 'ALIGN'):
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

    def test_align_grasp(self, out_dir, scenario):
        # S16~S22: mock이 받은 Align/Grasp 목표, 재정렬 기록, 단계별 /cmd_vel 입력
        if scenario not in FULL:
            return
        standoffs, grasp_calls, stage, realign = FULL[scenario]
        record, states, cmds = _load(out_dir)
        with open(os.path.join(out_dir, 'align_goals.csv')) as f:
            aligns = list(csv.DictReader(f))
        self.assertEqual([a['target_class'] for a in aligns], ['block'] * len(standoffs))
        for a, sd in zip(aligns, standoffs):
            self.assertAlmostEqual(float(a['standoff_m']), sd, delta=1e-9)
            self.assertAlmostEqual(float(a['timeout_sec']), 3.0, delta=1e-9)   # full.yaml
        self.assertEqual(len(aligns), len(standoffs))
        self.assertEqual(record['standoffs_used'], standoffs)
        self.assertEqual(record['realign_count'], realign)
        self.assertEqual(record['failed_stage'], stage)
        self.assertFalse(record['nav_only'])

        path = os.path.join(out_dir, 'grasp_goals.csv')
        grasps = list(csv.DictReader(open(path))) if os.path.exists(path) else []
        self.assertEqual(len(grasps), grasp_calls)
        for g in grasps:
            # mock_target_pose 값이 그대로 (base_link, 정렬 뒤 찍힌 1 s 이내 값)
            self.assertEqual((g['target_class'], g['frame_id']), ('block', 'base_link'))
            self.assertAlmostEqual(float(g['x']), TARGET_X[scenario], delta=1e-6)
            # 노드마다 /clock을 받는 시점이 한 스텝(0.01 s)쯤 달라 약간 음수가 될 수 있음
            self.assertGreaterEqual(float(g['age_sec']), -0.05)
            self.assertLessEqual(float(g['age_sec']), 1.0)

        # 정렬 중에는 /servoing/cmd_vel(SERVO_VX)이 /cmd_vel로 나간다
        align_windows = []
        for (t, _, s), nxt in zip(states, states[1:] + [(float('inf'), 0, '')]):
            if s == 'ALIGN':
                align_windows.append((t, nxt[0]))
        servo = [vx for t, _, vx, _ in cmds
                 if any(a + 0.3 <= t <= b - 0.15 for a, b in align_windows)]
        if scenario != 's20_align_timeout':
            self.assertTrue(servo and all(abs(v - SERVO_VX) < 1e-6 for v in servo), servo[:5])

    def test_pregrasp_request(self, proc_output, task_manager, out_dir, scenario):
        # 결과 기록과 경고 로그가 같은 조건에서만 나오는지 (nav_only 시나리오 · 먼 물체 · 물체 없음은 0번)
        record, _, _ = _load(out_dir)
        expected = PREGRASP_DISTANCE[scenario]
        got = record['pregrasp_request_distance_m']
        if expected is None:
            self.assertIsNone(got)
        else:
            self.assertAlmostEqual(got, expected, delta=1e-6)
        text = ''.join(io.text.decode(errors='replace') for io in proc_output[task_manager])
        self.assertEqual(text.count(NOT_SENT_LOG), 0 if expected is None else 1)

    def test_nav_goal_mode(self, out_dir, scenario):
        record, _, _ = _load(out_dir)
        self.assertEqual(record['nav_goal_mode'], 'goal' if scenario in MISSION_BASE else 'object')
        if scenario in ('s13_object_diagonal', 's14_object_behind', 's15_object_close'):
            # teleport로 계산한 목표에 도착 → 물체까지 0.40 m, 물체를 바라봄
            self.assertAlmostEqual(record['approach_distance'], _D, delta=1e-9)
            self.assertAlmostEqual(record['arrival']['object_dist'], _D, delta=0.005)
            self.assertLess(record['arrival']['dist'], 0.005)
