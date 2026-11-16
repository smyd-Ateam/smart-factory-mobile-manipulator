import math

import pytest

from mm_task_manager.pose_transition import (NEAR_TO_PREGRASP_THRESHOLD_M, PRE_GRASP,
                                             horizontal_distance, needs_pregrasp)


def test_threshold_is_provisional_035():
    # 확정 전 값 (PR #1 답변 대기). 값이 바뀌면 이 테스트도 같이 고친다
    assert NEAR_TO_PREGRASP_THRESHOLD_M == 0.35


def test_pre_grasp_pose_name():
    assert PRE_GRASP == 'pre_grasp'


def test_horizontal_distance_ignores_height():
    # base_link 기준 target_pose의 x, y만 쓴다 (interfaces.md 5.6: 로봇 중심 ~ 물체 중심 수평거리)
    assert horizontal_distance(0.3, 0.4) == pytest.approx(0.5)


@pytest.mark.parametrize('distance, expected', [
    (0.60, False),
    (0.35, False),      # 기준값과 같으면 아직 전환 안 함 ("아래로 내려오면")
    (0.349, True),
    (0.20, True),
    (math.nan, False),  # 잘못된 값으로는 전환하지 않음
])
def test_needs_pregrasp(distance, expected):
    assert needs_pregrasp(distance, 0.35) is expected


def test_needs_pregrasp_uses_given_threshold():
    assert needs_pregrasp(0.40, 0.45) is True
    assert needs_pregrasp(0.40, 0.30) is False
