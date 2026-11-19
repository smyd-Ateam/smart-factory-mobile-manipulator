import pytest

from mm_task_manager.start_gate import SubscriberWait, can_start, waiting_for_subscribers


def test_starts_when_autostart_and_clock_running():
    # 기본값(구독자 0개 대기)은 예전과 같다: autostart이고 sim time이 흐르면 바로 출발
    assert can_start(autostart=True, now=0.1, state_subscribers=0, required_subscribers=0)


def test_waits_for_clock():
    assert not can_start(autostart=True, now=0.0, state_subscribers=1, required_subscribers=0)


def test_no_autostart_never_starts():
    assert not can_start(autostart=False, now=1.0, state_subscribers=5, required_subscribers=0)


def test_waits_until_required_state_subscribers_matched():
    # mock 시험: recorder가 /task_manager/state에 붙기 전에 출발하면 WAIT_READY까지 놓칠 수 있다
    assert not can_start(autostart=True, now=1.0, state_subscribers=0, required_subscribers=1)
    assert can_start(autostart=True, now=1.0, state_subscribers=1, required_subscribers=1)
    assert can_start(autostart=True, now=1.0, state_subscribers=2, required_subscribers=1)


# ---- 구독자 대기 경고 (#43): 요구한 구독자가 안 붙으면 로그 없이 IDLE에서 계속 기다리던 문제 ----

@pytest.mark.parametrize('autostart, now, subs, required, waiting', [
    (True, 1.0, 0, 1, True),      # 구독자 때문에만 못 나감
    (True, 1.0, 1, 2, True),
    (True, 1.0, 1, 1, False),     # 다 붙음 → 출발
    (True, 1.0, 0, 0, False),     # 기본값 0: 기다리지 않음
    (True, 0.0, 0, 1, False),     # /clock 대기는 구독자 대기가 아님
    (False, 1.0, 0, 1, False),    # autostart 꺼짐: 원래 출발 안 함
])
def test_waiting_for_subscribers(autostart, now, subs, required, waiting):
    assert waiting_for_subscribers(autostart, now, subs, required) is waiting


def test_no_warning_before_period():
    w = SubscriberWait(period_sec=5.0)
    assert w.update(now=100.0, waiting=True) is None
    assert w.update(now=104.9, waiting=True) is None


def test_warning_after_period_then_every_period():
    # 계속 기다리면 5 s, 10 s, 15 s ...에 한 번씩 (기다린 시간을 돌려줌), 그 사이엔 없음
    w = SubscriberWait(period_sec=5.0)
    w.update(now=100.0, waiting=True)
    assert w.update(now=105.0, waiting=True) == pytest.approx(5.0)
    assert w.update(now=107.0, waiting=True) is None
    assert w.update(now=110.0, waiting=True) == pytest.approx(10.0)
    assert w.update(now=110.1, waiting=True) is None
    assert w.update(now=115.2, waiting=True) == pytest.approx(15.2)


def test_reset_when_wait_ends():
    # 기다림이 끝났다가 다시 시작하면 처음부터 5 s
    w = SubscriberWait(period_sec=5.0)
    w.update(now=100.0, waiting=True)
    assert w.update(now=104.0, waiting=False) is None
    assert w.update(now=106.0, waiting=True) is None
    assert w.update(now=110.9, waiting=True) is None
    assert w.update(now=111.0, waiting=True) == pytest.approx(5.0)


def test_never_warns_when_not_waiting():
    w = SubscriberWait(period_sec=5.0)
    assert all(w.update(now=float(t), waiting=False) is None for t in range(0, 60))


@pytest.mark.parametrize('bad', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_period_rejected(bad):
    with pytest.raises(ValueError):
        SubscriberWait(period_sec=bad)
