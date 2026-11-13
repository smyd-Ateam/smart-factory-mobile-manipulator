"""목표 근처 맴돌기 회귀 테스트 (Gazebo + Nav2, 약 2~3분).

MM_SIM_TEST=1 일 때만 시뮬을 띄운다. 없으면 바로 skip (기본 colcon test는 mock만).
  MM_SIM_TEST=1 colcon test --packages-select mm_navigation --event-handlers console_direct+

시나리오: 다가오는 방향과 목표 yaw가 거의 반대(약 180도)라 도착 후 제자리 회전이 필요한 목표.
  - standalone : Nav2 출력이 /cmd_vel로 바로, 테스트가 목표 2개를 직접 보냄
  - task_manager: 실제 운영처럼 상태머신이 목표를 보내고 /nav2/cmd_vel → /cmd_vel 중계

판정 (map→base_link가 목표 0.10 m 안에 처음 들어온 때부터 이동 단계 끝까지)
  - 걸린 시간(sim time) <= DONE_LIMIT_SEC
  - 총 회전량(odom→base_link yaw 변화 합) <= ROT_FACTOR × 필요 회전량
    필요 회전량 = 그때 목표 yaw 오차 − yaw 허용치(0.25)
"""
import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest

ENABLED = os.environ.get('MM_SIM_TEST') == '1'

IN_WINDOW_M = 0.10          # Nav2 xy_goal_tolerance
YAW_TOL = 0.25              # Nav2 yaw_goal_tolerance
DONE_LIMIT_SEC = 7.0             # 맴돌지 않고 한 번에 돌아도 5.1~5.9 s 걸림 (2026-11-13 측정)
ROT_FACTOR = 1.5
NAV_WAIT_SEC = 120.0        # wall, 목표 하나당

# 스폰 (0, 0, 0) → (1, 0)에 yaw π로 도착 → 다시 (0, 0)에 yaw 0으로 도착 (둘 다 약 180도 회전)
GOALS_STANDALONE = [(1.0, 0.0, math.pi), (0.0, 0.0, 0.0)]
GOAL_TASK_MANAGER = (1.0, 0.0, math.pi)


def _kill_leftover_gazebo():
    """launch 종료 때 ruby 래퍼(ign gazebo)만 끝나고 서버가 남는 경우가 있어 직접 정리한다.

    남은 서버가 다음 실행과 /clock을 같이 내보내면 측정이 틀어진다 (2026-11-13 확인).
    test_room.sdf를 띄운 ign gazebo만 대상.
    """
    import signal
    import subprocess

    def pids():
        out = subprocess.run(['pgrep', '-f', 'ign gazebo .*test_room.sdf'],
                             capture_output=True, text=True).stdout
        return [int(p) for p in out.split()]

    for sig, wait in ((signal.SIGINT, 3.0), (signal.SIGKILL, 1.0)):
        found = pids()
        if not found:
            return 0
        for p in found:
            try:
                os.kill(p, sig)
            except ProcessLookupError:
                pass
        time.sleep(wait)
    return len(pids())


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


@pytest.mark.launch_test
@launch_testing.parametrize('mode', ['standalone', 'task_manager'])
def generate_test_description(mode):
    if not ENABLED:
        # 할 일이 없으면 launch가 바로 끝나 'Launch stopped before the active tests finished'가 나므로
        # 테스트가 끝날 때까지 빈 타이머로 살려 둔다 (테스트 종료 시 취소됨)
        return launch.LaunchDescription([
            TimerAction(period=600.0, actions=[]),
            launch_testing.actions.ReadyToTest(),
        ]), {'mode': mode}

    assert _kill_leftover_gazebo() == 0, '이전 ign gazebo 서버가 남아 있음'
    nav =IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mm_navigation'), 'launch', 'nav2.launch.py')),
        launch_arguments={
            'software_gl': 'true', 'headless': 'true', 'rviz': 'false',
            'standalone': 'true' if mode == 'standalone' else 'false',
        }.items(),
    )
    actions = [nav]
    if mode == 'task_manager':
        mission = os.path.join(os.path.dirname(__file__), 'mission_spin_at_goal.yaml')
        actions.append(TimerAction(period=5.0, actions=[Node(
            package='mm_task_manager', executable='task_manager', name='task_manager',
            parameters=[mission, {'use_sim_time': True}], output='screen')]))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {'mode': mode}


class TestSpinAtGoal(unittest.TestCase):

    def test_spin_at_goal(self, mode):
        if not ENABLED:
            self.skipTest('MM_SIM_TEST=1 일 때만 실행 (Gazebo + Nav2 필요)')
        import rclpy
        rclpy.init()
        try:
            probe = _Probe()
            try:
                if mode == 'standalone':
                    results = [probe.run_goal(g) for g in GOALS_STANDALONE]
                else:
                    results = [probe.watch_task_manager(GOAL_TASK_MANAGER)]
            finally:
                probe.destroy_node()
        finally:
            rclpy.shutdown()

        for i, r in enumerate(results):
            print(f'[{mode}] goal{i} {r}')
        for i, r in enumerate(results):
            with self.subTest(goal=i):
                self.assertTrue(r['succeeded'], r)
                self.assertIsNotNone(r['entered'], f'0.10 m 안에 못 들어옴: {r}')
                self.assertLessEqual(r['done_sec'], DONE_LIMIT_SEC, r)
                self.assertLessEqual(r['rotation'], ROT_FACTOR * r['required'], r)


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):

    def test_no_leftover_gazebo(self, mode):
        if not ENABLED:
            self.skipTest('MM_SIM_TEST=1 일 때만 실행')
        self.assertEqual(_kill_leftover_gazebo(), 0)


class _Probe:
    """map→base_link, odom→base_link를 20 Hz로 보고 목표 근처 구간을 측정."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node as RosNode
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        import tf2_ros

        self._rclpy = rclpy
        self._NavigateToPose = NavigateToPose
        self.node = RosNode('spin_at_goal_probe', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self.node)
        self.ac = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.state = None
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/task_manager/state', self._on_state, qos)
        self.goal = None
        self.track = None
        self.node.create_timer(0.05, self._tick)

    def destroy_node(self):
        self.node.destroy_node()

    def _on_state(self, msg):
        self.state = msg.data

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _pose(self, parent):
        from rclpy.time import Time
        t = self.buf.lookup_transform(parent, 'base_link', Time())
        p = t.transform.translation
        return p.x, p.y, _yaw(t.transform.rotation)

    def _tick(self):
        if self.goal is None or self.track is None:
            return
        try:
            mx, my, myaw = self._pose('map')
            _, _, oyaw = self._pose('odom')
        except Exception:  # noqa: BLE001  TF가 아직 없을 때
            return
        gx, gy, gyaw = self.goal
        tr = self.track
        if tr['entered'] is None and math.hypot(gx - mx, gy - my) < IN_WINDOW_M:
            tr['entered'] = self._now()
            tr['required'] = max(abs(_wrap(gyaw - myaw)) - YAW_TOL, 0.0)
            tr['last_oyaw'] = oyaw
        elif tr['entered'] is not None and tr['done'] is None:
            tr['rotation'] += abs(_wrap(oyaw - tr['last_oyaw']))
            tr['last_oyaw'] = oyaw

    def _spin_until(self, cond, timeout):
        t0 = time.time()
        while not cond() and time.time() - t0 < timeout:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
        return cond()

    def _start(self, goal):
        self.goal = goal
        self.track = {'entered': None, 'done': None, 'rotation': 0.0, 'required': None}

    def _finish(self, succeeded):
        tr = self.track
        tr['done'] = self._now()
        return {
            'succeeded': succeeded,
            'entered': tr['entered'],
            'done_sec': round(tr['done'] - tr['entered'], 2) if tr['entered'] else None,
            'rotation': round(tr['rotation'], 2),
            'required': round(tr['required'], 2) if tr['required'] is not None else None,
        }

    def _wait_nav_ready(self):
        assert self.ac.wait_for_server(timeout_sec=120.0), 'navigate_to_pose 서버 없음'

        def tf_ok():
            try:
                self._pose('map')
                return True
            except Exception:  # noqa: BLE001
                return False
        assert self._spin_until(tf_ok, 120.0), 'map→base_link 없음'
        self._spin_until(lambda: False, 2.0)

    def run_goal(self, goal):
        self._wait_nav_ready()
        g = self._NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.node.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = goal[0], goal[1]
        g.pose.pose.orientation.z = math.sin(goal[2] / 2)
        g.pose.pose.orientation.w = math.cos(goal[2] / 2)
        self._start(goal)
        fut = self.ac.send_goal_async(g)
        assert self._spin_until(fut.done, 30.0), '목표 전송 응답 없음'
        handle = fut.result()
        assert handle.accepted, '목표 거부'
        res = handle.get_result_async()
        self._spin_until(res.done, NAV_WAIT_SEC)
        if not res.done():
            handle.cancel_goal_async()
            return self._finish(False)
        out = self._finish(res.result().status == 4)   # STATUS_SUCCEEDED
        self._spin_until(lambda: False, 2.0)
        return out

    def watch_task_manager(self, goal):
        self._start(goal)
        assert self._spin_until(lambda: self.state == 'NAVIGATE', 150.0), \
            f'task_manager가 NAVIGATE로 안 감 (state={self.state})'
        self._spin_until(lambda: self.state != 'NAVIGATE', NAV_WAIT_SEC)
        nav_done = self.state != 'NAVIGATE'
        out = self._finish(nav_done)
        # 이동 단계 이후 최종 결과까지 확인
        self._spin_until(lambda: self.state in ('SUCCEEDED', 'FAILED'), 30.0)
        out['succeeded'] = nav_done and self.state == 'SUCCEEDED'
        out['final_state'] = self.state
        return out
