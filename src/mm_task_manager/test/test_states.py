from mm_task_manager.states import State, StateReporter

# interfaces.md v0.14 3장 /task_manager/state 단계 이름
DOC_NAMES = ['IDLE', 'WAIT_READY', 'NAVIGATE', 'VERIFY_ARRIVAL', 'ALIGN', 'GRASP',
             'CANCELING', 'SUCCEEDED', 'FAILED']


def test_state_names_match_interfaces_doc():
    assert [s.value for s in State] == DOC_NAMES


def test_first_update_reports():
    r = StateReporter()
    assert r.update(State.IDLE) == 'IDLE'


def test_reports_only_on_change():
    r = StateReporter()
    r.update(State.IDLE)
    assert r.update(State.IDLE) is None
    assert r.update(State.WAIT_READY) == 'WAIT_READY'
    assert r.update(State.WAIT_READY) is None
    assert r.update(State.NAVIGATE) == 'NAVIGATE'


def test_realign_back_to_align_reports_again():
    r = StateReporter()
    for s in (State.ALIGN, State.GRASP):
        r.update(s)
    assert r.update(State.ALIGN) == 'ALIGN'
