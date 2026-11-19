"""관측 자세 액션 클라이언트 (제안 초안, B 확인 전) + mock 서버 단독 시험.

ObservePoseClient가 /arm/observe_pose로 pose_name을 보내고, mock_observe_pose_server가
duration_sec(0.5 s) 뒤 success를 돌려주는지 확인한다. 상태머신 흐름에는 아직 연결하지 않았다.
자세 이름은 far, near, pre_grasp (interfaces.md 5.6). mock은 그 밖의 이름을 거부한다.
ROS가 필요하다 (source install/setup.bash).
"""

import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mm_task_manager import codes
from mm_task_manager.mocks.mock_observe_pose_server import MockObservePoseServer
from mm_task_manager.observe_pose_client import POSE_NAMES, ObservePoseClient


@pytest.fixture
def ros():
    rclpy.init()
    server = MockObservePoseServer()
    node = Node('observe_pose_client_test')
    client = ObservePoseClient(node)
    ex = MultiThreadedExecutor()
    ex.add_node(server)
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    yield client
    server.closing = True
    ex.shutdown()
    node.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


def _send(client, pose_name, timeout=5.0):
    assert client.wait_for_server(timeout_sec=5.0)
    done = threading.Event()
    got = {}

    def on_done(success, error_code, detail):
        got.update(success=success, error_code=error_code, detail=detail, t=time.monotonic())
        done.set()

    t0 = time.monotonic()
    client.send(pose_name, on_done)
    assert done.wait(timeout), 'observe_pose 결과가 오지 않음'
    got['elapsed'] = got['t'] - t0
    return got


def test_pose_names():
    # 근접 정렬 2단계(near에서 1차 Align → pre_grasp로 전환 → 2차 Align) 대비 pre_grasp 추가
    assert POSE_NAMES == ('far', 'near', 'pre_grasp')


@pytest.mark.parametrize('pose_name', ['far', 'near', 'pre_grasp'])
def test_known_pose_succeeds(ros, pose_name):
    got = _send(ros, pose_name)
    assert (got['success'], got['error_code'], got['detail']) == (True, '', '')


@pytest.mark.parametrize('pose_name', ['grasp', 'pre-grasp', ''])
def test_unknown_pose_is_rejected(ros, pose_name):
    # 목표 거부는 Align/Grasp와 같은 규칙: UNKNOWN / rejected
    got = _send(ros, pose_name)
    assert (got['success'], got['error_code'], got['detail']) == (
        False, codes.UNKNOWN, codes.REJECTED)


def test_result_comes_after_mock_delay(ros):
    # mock은 duration_sec(기본 0.5 s) 뒤에 결과를 준다
    got = _send(ros, 'far')
    assert 0.4 <= got['elapsed'] < 3.0
