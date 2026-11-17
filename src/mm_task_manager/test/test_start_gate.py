from mm_task_manager.start_gate import can_start


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
