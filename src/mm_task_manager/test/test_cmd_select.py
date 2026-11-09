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
