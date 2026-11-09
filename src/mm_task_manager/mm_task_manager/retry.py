"""IK 실패 후 재정렬 판정 (docs/interfaces.md 5.2)."""

from mm_task_manager import codes

REALIGN = 'realign'
FAIL = 'fail'


def after_grasp_failure(code, realign_count, max_realign):
    """Grasp 실패 후 다음 행동. realign_count = 지금까지 한 재정렬 횟수."""
    if code == codes.IK_FAILED and realign_count < max_realign:
        return REALIGN
    return FAIL


def standoff_for(attempt, standoff_list):
    """attempt: 0 = 첫 정렬, 1 = 첫 재정렬, ..."""
    if attempt < 0 or attempt >= len(standoff_list):
        raise ValueError(f'no standoff for attempt {attempt} (list has {len(standoff_list)})')
    return standoff_list[attempt]
