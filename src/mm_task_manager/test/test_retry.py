import pytest

from mm_task_manager import codes
from mm_task_manager.retry import FAIL, REALIGN, after_grasp_failure, standoff_for


@pytest.mark.parametrize('realign_count, expected', [
    (0, REALIGN),
    (1, REALIGN),
    (2, FAIL),     # 재정렬 2회 후 3번째 실패 → 최종 실패 (5.2)
])
def test_ik_failed_realigns_up_to_max(realign_count, expected):
    assert after_grasp_failure(codes.IK_FAILED, realign_count, max_realign=2) == expected


@pytest.mark.parametrize('code', [
    codes.GRASP_MISS, codes.GRASP_SLIP, codes.OBJECT_NOT_FOUND, codes.UNKNOWN, '',
])
def test_other_failures_do_not_realign(code):
    assert after_grasp_failure(code, 0, max_realign=2) == FAIL


def test_max_realign_zero_never_realigns():
    assert after_grasp_failure(codes.IK_FAILED, 0, max_realign=0) == FAIL


def test_standoff_for_each_attempt():
    standoffs = [0.25, 0.22, 0.28]
    assert [standoff_for(i, standoffs) for i in range(3)] == [0.25, 0.22, 0.28]


def test_standoff_for_out_of_range_raises():
    with pytest.raises(ValueError):
        standoff_for(3, [0.25, 0.22, 0.28])
    with pytest.raises(ValueError):
        standoff_for(-1, [0.25])
