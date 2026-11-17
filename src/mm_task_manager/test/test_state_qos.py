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

from mm_task_manager.state_qos import STATE_QOS_DEPTH, state_qos

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
