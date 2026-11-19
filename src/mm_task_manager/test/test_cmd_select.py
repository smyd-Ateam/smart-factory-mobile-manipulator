import pytest

from mm_task_manager.cmd_select import NAV, SERVO, ZERO, CmdSelector, source_for
from mm_task_manager.states import State


@pytest.mark.parametrize('state, source', [
    (State.IDLE, ZERO),
    (State.WAIT_READY, ZERO),
    (State.NAVIGATE, NAV),
    (State.VERIFY_ARRIVAL, ZERO),
    (State.ALIGN, SERVO),
    (State.GRASP, ZERO),
    (State.CANCELING, ZERO),
    (State.SUCCEEDED, ZERO),
    (State.FAILED, ZERO),
])
def test_source_for_matches_interfaces_5_0(state, source):
    assert source_for(state) == source


def test_forwards_selected_input():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(NAV, now=0.0)
    sel.update(NAV, 'nav_cmd', now=1.0)
    sel.update(SERVO, 'servo_cmd', now=1.0)
    assert sel.output(now=1.1) == 'nav_cmd'


def test_other_input_is_not_forwarded():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(SERVO, now=0.0)
    sel.update(NAV, 'nav_cmd', now=1.0)
    assert sel.output(now=1.1) is None


def test_zero_source_outputs_none():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(ZERO, now=0.0)
    sel.update(NAV, 'nav_cmd', now=1.0)
    assert sel.output(now=1.0) is None


def test_no_input_received_outputs_none():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(NAV, now=0.0)
    assert sel.output(now=0.1) is None


def test_input_timeout_outputs_none():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(NAV, now=0.0)
    sel.update(NAV, 'nav_cmd', now=1.0)
    assert sel.output(now=1.5) == 'nav_cmd'   # 경계값은 아직 유효
    assert sel.output(now=1.51) is None


def test_input_received_before_switch_is_ignored():
    # 이전 단계에서 받은 명령이 단계가 바뀐 직후에 흘러나가지 않게
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(ZERO, now=0.0)
    sel.update(SERVO, 'old_servo_cmd', now=1.0)
    sel.select(SERVO, now=1.2)
    assert sel.output(now=1.3) is None
    sel.update(SERVO, 'new_servo_cmd', now=1.3)
    assert sel.output(now=1.3) == 'new_servo_cmd'


def test_select_same_source_does_not_reset():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(NAV, now=0.0)
    sel.update(NAV, 'nav_cmd', now=1.0)
    sel.select(NAV, now=1.1)
    assert sel.output(now=1.1) == 'nav_cmd'


def test_unknown_source_raises():
    sel = CmdSelector(input_timeout_sec=0.5)
    with pytest.raises(ValueError):
        sel.select('joystick', now=0.0)
    with pytest.raises(ValueError):
        sel.update('joystick', 'x', now=0.0)


# ---- 1. 같은 소스로 계속 받을 때: 매번 최신 값으로 덮어씀 ----

def test_repeated_updates_overwrite_previous_command():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(NAV, now=0.0)
    for i in range(5):
        sel.update(NAV, f'nav_cmd_{i}', now=1.0 + 0.1 * i)
        assert sel.output(now=1.0 + 0.1 * i) == f'nav_cmd_{i}'


def test_repeated_select_same_state_keeps_latest_and_timeout_from_latest():
    # 같은 state(같은 소스)로 select가 매 주기 반복돼도 이전 값이 남지 않고, 끊김 판정은 최신 수신 시각 기준
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(SERVO, now=0.0)
    sel.update(SERVO, 'servo_old', now=1.0)
    sel.select(SERVO, now=1.1)
    sel.update(SERVO, 'servo_new', now=1.2)
    sel.select(SERVO, now=1.3)
    assert sel.output(now=1.6) == 'servo_new'    # 1.0 기준이면 이미 끊겼을 시각
    assert sel.output(now=1.71) is None          # 1.2 + 0.5 지나면 끊김


# ---- 2. SERVO 입력 끊김 ----

def test_servo_input_timeout_outputs_none():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(SERVO, now=0.0)
    sel.update(SERVO, 'servo_cmd', now=1.0)
    assert sel.output(now=1.5) == 'servo_cmd'    # 경계값은 아직 유효
    assert sel.output(now=1.51) is None


def test_servo_timeout_not_rescued_by_nav_input():
    # SERVO가 끊긴 동안 NAV가 들어와도 대신 내보내지 않음
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(SERVO, now=0.0)
    sel.update(SERVO, 'servo_cmd', now=1.0)
    sel.update(NAV, 'nav_cmd', now=1.6)
    assert sel.output(now=1.6) is None


def test_servo_recovers_after_timeout_when_new_input_arrives():
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(SERVO, now=0.0)
    sel.update(SERVO, 'servo_cmd', now=1.0)
    assert sel.output(now=2.0) is None
    sel.update(SERVO, 'servo_cmd_2', now=2.1)
    assert sel.output(now=2.1) == 'servo_cmd_2'


# ---- 3. NAV → ZERO → NAV 반복 전환 ----

def test_nav_zero_nav_cycles_pick_right_source_each_time():
    sel = CmdSelector(input_timeout_sec=0.5)
    t = 0.0
    for cycle in range(3):
        sel.select(NAV, now=t)
        assert sel.selected == NAV
        assert sel.output(now=t) is None                  # 이전 NAV 주기 값은 안 나감
        sel.update(NAV, f'nav_{cycle}', now=t + 0.1)
        assert sel.output(now=t + 0.1) == f'nav_{cycle}'

        sel.select(ZERO, now=t + 0.2)
        assert sel.selected == ZERO
        sel.update(NAV, f'nav_late_{cycle}', now=t + 0.3)  # ZERO 중에 들어온 NAV
        assert sel.output(now=t + 0.3) is None
        t += 1.0


def test_nav_command_received_during_zero_is_not_used_after_switch_back():
    # ZERO 동안 받은 NAV 명령도 '전환 전에 받은 값'이라 다시 NAV가 된 직후 내보내지 않음
    sel = CmdSelector(input_timeout_sec=0.5)
    sel.select(NAV, now=0.0)
    sel.select(ZERO, now=1.0)
    sel.update(NAV, 'nav_during_zero', now=1.1)
    sel.select(NAV, now=1.2)
    assert sel.output(now=1.25) is None
    sel.update(NAV, 'nav_fresh', now=1.3)
    assert sel.output(now=1.3) == 'nav_fresh'


# ---- 4. 타임아웃 설정값 검증: 0 · 음수 · NaN · inf · 숫자 아닌 값은 거부 ----

@pytest.mark.parametrize('bad', [
    0, 0.0, -0.1, -1,
    float('nan'), float('inf'), float('-inf'),
    '0.5', 'abc', None, True,
])
def test_invalid_input_timeout_raises(bad):
    with pytest.raises(ValueError):
        CmdSelector(input_timeout_sec=bad)


@pytest.mark.parametrize('ok', [0.5, 1, 1e-3])
def test_valid_input_timeout_accepted(ok):
    sel = CmdSelector(input_timeout_sec=ok)
    sel.select(NAV, now=0.0)
    sel.update(NAV, 'nav_cmd', now=1.0)
    assert sel.output(now=1.0) == 'nav_cmd'
