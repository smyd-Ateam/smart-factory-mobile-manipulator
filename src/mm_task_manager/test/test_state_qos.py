"""/task_manager/state QoS: 늦게 붙은 구독자도 앞 단계(IDLE 등)를 받는지 (실제 rclpy 발행 · 구독).

transient_local은 발행자가 depth만큼 이력을 들고 있다가 나중에 붙은 구독자에게 보낸다.
depth 1이면 IDLE이 WAIT_READY에 덮여 늦은 구독자는 WAIT_READY만 받는다 (mock 시험에서 실제로 발생).
"""

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from mm_task_manager import codes
from mm_task_manager.machine import (AlignResult, CancelDone, GraspResult, SendAlignGoal,
                                     SendGraspGoal, Tick)
from mm_task_manager.state_qos import STATE_QOS_DEPTH, state_qos
from mm_task_manager.states import State, StateReporter

from test_machine import make, to_grasp

TOPIC = '/test_state_qos/state'


@pytest.fixture
def ctx():
    context = rclpy.Context()
    rclpy.init(context=context)
    yield context
    rclpy.shutdown(context=context)


def _late_subscriber_receives(ctx, pub_qos, sub_qos, sent, expect_count, timeout=5.0):
    pub_node = rclpy.create_node('state_qos_pub', context=ctx)
    sub_node = rclpy.create_node('state_qos_sub', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(pub_node)
    executor.add_node(sub_node)
    try:
        pub = pub_node.create_publisher(String, TOPIC, pub_qos)
        for name in sent:                     # 구독자가 생기기 전에 발행
            pub.publish(String(data=name))
        got = []
        sub_node.create_subscription(String, TOPIC, lambda m: got.append(m.data), sub_qos)
        end = time.monotonic() + timeout
        while time.monotonic() < end and len(got) < expect_count:
            executor.spin_once(timeout_sec=0.05)
        # 더 올 것이 없는지 잠깐 더 받는다
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        return got
    finally:
        executor.shutdown()
        pub_node.destroy_node()
        sub_node.destroy_node()


def test_depth_one_loses_idle_for_late_subscriber(ctx):
    # 예전 설정 (비교용): 늦은 구독자는 마지막 단계만 받는다
    old = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    assert _late_subscriber_receives(ctx, old, old, ['IDLE', 'WAIT_READY'], 1) == ['WAIT_READY']


def test_state_qos_keeps_idle_for_late_subscriber(ctx):
    got = _late_subscriber_receives(ctx, state_qos(), state_qos(), ['IDLE', 'WAIT_READY'], 2)
    assert got == ['IDLE', 'WAIT_READY']


def test_state_qos_keeps_first_steps_of_a_run(ctx):
    # 측정상 recorder가 가장 늦게 붙은 경우 약 3 s: 그 사이 IDLE · WAIT_READY · NAVIGATE (+ 여유)
    sent = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'VERIFY_ARRIVAL', 'ALIGN']
    assert _late_subscriber_receives(ctx, state_qos(), state_qos(), sent, len(sent)) == sent


def test_state_qos_is_transient_local():
    q = state_qos()
    assert q.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert q.depth == STATE_QOS_DEPTH >= 2


# ---- 한 실행에서 나올 수 있는 최대 상태 메시지 수 (재정렬 최대 + 시간 제한 취소) ----
# 전환: IDLE → WAIT_READY → NAVIGATE → VERIFY_ARRIVAL → (ALIGN → GRASP) × (max_realign + 1)
#       → 마지막 GRASP 시간 제한 → CANCELING → FAILED.  되돌아가는 반복은 GRASP → ALIGN(재정렬)뿐
#       → 최대 2 * max_realign + 8개 (기본 max_realign 2 → 12개)

def _max_run_published_states(max_realign=None):
    """상태머신을 최대 경로로 돌리며, 노드처럼 StateReporter로 바뀔 때만 모은 발행 목록."""
    overrides = dict(nav_only=False)
    if max_realign is not None:
        overrides.update(max_realign=max_realign, standoff_list=[0.25] * (max_realign + 1))
    m = make(**overrides)
    reporter = StateReporter()
    published = [reporter.update(m.state)]
    handle = m.handle

    def reporting_handle(ev):
        cmds = handle(ev)
        name = reporter.update(m.state)
        if name is not None:
            published.append(name)
        return cmds

    m.handle = reporting_handle
    grasp = to_grasp(m)
    t = 30.0
    for _ in range(m.m.max_realign):
        cmds = m.handle(GraspResult(now=t, token=grasp.token, success=False,
                                    error_code=codes.IK_FAILED))
        align = [c for c in cmds if isinstance(c, SendAlignGoal)][0]
        cmds = m.handle(AlignResult(now=t + 1.0, token=align.token, success=True))
        grasp = [c for c in cmds if isinstance(c, SendGraspGoal)][0]
        t += 10.0
    m.handle(Tick(now=t + m.m.grasp_timeout_sec + 1.0))       # 마지막 파지 시간 제한
    assert m.state == State.CANCELING
    m.handle(CancelDone(now=t + m.m.grasp_timeout_sec + 2.0, token=grasp.token))
    assert m.state == State.FAILED
    return published


def test_max_run_publishes_twelve_states_with_default_realign():
    assert _max_run_published_states() == [
        'IDLE', 'WAIT_READY', 'NAVIGATE', 'VERIFY_ARRIVAL',
        'ALIGN', 'GRASP', 'ALIGN', 'GRASP', 'ALIGN', 'GRASP',
        'CANCELING', 'FAILED']


@pytest.mark.parametrize('max_realign', [0, 1, 2, 3, 4])
def test_max_run_message_count_is_2r_plus_8(max_realign):
    assert len(_max_run_published_states(max_realign)) == 2 * max_realign + 8


def test_depth_covers_max_run_with_margin():
    # 기본 설정(max_realign 2) 최대 12개 + 여유 5
    assert STATE_QOS_DEPTH >= len(_max_run_published_states()) + 5


def test_depth_ten_loses_idle_on_max_run(ctx):
    # 예전 depth 10 (비교용): 최대 경로 12개면 늦은 구독자는 마지막 10개만 받아 IDLE · WAIT_READY를 잃는다
    sent = _max_run_published_states()
    old = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got = _late_subscriber_receives(ctx, old, old, sent, 10)
    assert got == sent[-10:] and 'IDLE' not in got


def test_state_qos_keeps_whole_max_run_for_late_subscriber(ctx):
    sent = _max_run_published_states()
    assert _late_subscriber_receives(ctx, state_qos(), state_qos(), sent, len(sent)) == sent
