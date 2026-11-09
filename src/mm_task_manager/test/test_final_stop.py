from mm_task_manager.final_stop import FinalStop


def test_not_started_does_nothing():
    fs = FinalStop(duration_sec=1.0)
    assert fs.started is False
    assert fs.should_exit(now=100.0) is False


def test_exit_after_duration():
    fs = FinalStop(duration_sec=1.0)
    fs.start(now=10.0)
    assert fs.should_exit(now=10.5) is False
    assert fs.should_exit(now=11.0) is True     # 경계값 포함


def test_start_only_once():
    # 종료 상태에서 이벤트가 더 와도 시작 시각이 밀리지 않게
    fs = FinalStop(duration_sec=1.0)
    fs.start(now=10.0)
    fs.start(now=10.8)
    assert fs.should_exit(now=11.0) is True


def test_zero_duration_exits_immediately():
    fs = FinalStop(duration_sec=0.0)
    fs.start(now=5.0)
    assert fs.should_exit(now=5.0) is True
