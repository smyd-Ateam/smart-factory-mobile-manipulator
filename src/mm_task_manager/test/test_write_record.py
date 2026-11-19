"""task_manager_node 결과 기록: 못 쓸 때 노드가 죽지 않는지, 기록 내용이 로그에 남는지.

결과 기록은 실행이 끝날 때(SUCCEEDED/FAILED) 한 번 쓴다. 여기서 예외가 나면 노드가 상태 발행과
final_stop(/cmd_vel 0을 final_zero_sec 동안 발행) 전에 죽을 수 있으므로, 어떤 예외든 밖으로 내지 않고
에러 로그(기록 내용 포함: JSON 한 줄, 안 되면 repr)를 남긴 뒤 "finished ..." 로그까지 이어지는지 본다.
노드가 뜰 때의 경로 확인(_check_result_log)은 경고만 하고 진행한다. rclpy 노드 없이 함수만 부른다.
"""

import json
import os
import types

import pytest

from mm_task_manager.task_manager_node import TaskManagerNode

RECORD = {'final_state': 'FAILED', 'error_code': 'NAV_FAILED', 'detail': 'timeout',
          'failed_stage': 'NAVIGATE'}


class _Logger:
    def __init__(self):
        self.lines = []

    def error(self, msg):
        self.lines.append(('error', msg))

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))


def _fake(path):
    logger = _Logger()
    fake = types.SimpleNamespace(mission=types.SimpleNamespace(result_log_path=path),
                                 get_logger=lambda: logger)
    return fake, logger


def _write(path, record=RECORD):
    fake, logger = _fake(path)
    TaskManagerNode._write_record(fake, dict(record))
    return logger.lines


def _startup_check(path):
    fake, logger = _fake(path)
    TaskManagerNode._check_result_log(fake)
    return logger.lines


def test_missing_folder_is_created(tmp_path):
    path = tmp_path / 'a' / 'b' / 'result.jsonl'
    lines = _write(str(path))
    assert json.loads(path.read_text())['final_state'] == 'FAILED'
    assert [lvl for lvl, _ in lines] == ['info']


def _unwritable(tmp_path, kind):
    if kind == 'read_only_dir':
        d = tmp_path / 'ro'
        d.mkdir()
        os.chmod(d, 0o555)
        return str(d / 'result.jsonl')
    if kind == 'no_permission_dir':
        d = tmp_path / 'locked'
        d.mkdir()
        os.chmod(d, 0o000)
        return str(d / 'sub' / 'result.jsonl')
    if kind == 'read_only_file':
        f = tmp_path / 'result.jsonl'
        f.write_text('')
        os.chmod(f, 0o444)
        return str(f)
    if kind == 'parent_is_file':
        f = tmp_path / 'file'
        f.write_text('')
        return str(f / 'result.jsonl')
    if kind == 'path_is_dir':
        return str(tmp_path)
    if kind == 'empty':
        return ''
    raise ValueError(kind)


@pytest.mark.parametrize('kind', ['read_only_dir', 'no_permission_dir', 'read_only_file',
                                  'parent_is_file', 'path_is_dir', 'empty'])
def test_unwritable_path_does_not_raise(tmp_path, kind):
    try:
        lines = _write(_unwritable(tmp_path, kind))
    finally:
        for p in tmp_path.iterdir():            # tmp_path 정리가 되도록 권한 복구
            os.chmod(p, 0o755)
    levels = [lvl for lvl, _ in lines]
    assert levels == ['error', 'info'], lines   # 에러 로그 뒤 "finished ..."까지 이어짐
    assert 'result log write failed' in lines[0][1]
    # 파일은 못 써도 기록 내용(JSON 한 줄)이 에러 로그에 남음
    assert '"final_state": "FAILED"' in lines[0][1]
    assert '"wall_time"' in lines[0][1]
    assert lines[1][1].startswith('finished FAILED')


def test_unserializable_value_does_not_raise(tmp_path):
    # 기록 값에 JSON으로 못 바꾸는 값이 섞이면 (지금 기록 필드에서는 나올 경로를 못 찾음, 방어용)
    path = tmp_path / 'result.jsonl'
    lines = _write(str(path), dict(RECORD, arrival={'pose': object()}))
    assert [lvl for lvl, _ in lines] == ['error', 'info'], lines
    assert 'result log write failed' in lines[0][1]
    # JSON으로 못 바꾸므로 repr로 남김
    assert "'final_state': 'FAILED'" in lines[0][1]
    assert lines[1][1].startswith('finished FAILED')
    # 반쯤 쓴 줄이 남지 않음 (다음 기록이 이어 붙어 깨지지 않게)
    assert not path.exists() or path.read_text() == ''


# ---- 노드가 뜰 때 경로 확인: 못 쓰면 에러 로그만 (실행은 막지 않음) ----

def test_startup_check_ok_for_new_folder(tmp_path):
    path = tmp_path / 'a' / 'b' / 'result.jsonl'
    assert _startup_check(str(path)) == []
    assert not path.exists()                     # 확인만 하고 빈 파일을 만들지 않음


def test_startup_check_ok_for_existing_file(tmp_path):
    path = tmp_path / 'result.jsonl'
    path.write_text('{"a": 1}\n')
    assert _startup_check(str(path)) == []
    assert path.read_text() == '{"a": 1}\n'       # 기존 기록을 건드리지 않음


@pytest.mark.parametrize('kind', ['read_only_dir', 'no_permission_dir', 'read_only_file',
                                  'parent_is_file', 'path_is_dir', 'empty'])
def test_startup_check_warns_when_unwritable(tmp_path, kind):
    try:
        path = _unwritable(tmp_path, kind)
        lines = _startup_check(path)
    finally:
        for p in tmp_path.iterdir():
            os.chmod(p, 0o755)
    assert [lvl for lvl, _ in lines] == ['error'], lines
    assert 'result log not writable' in lines[0][1]
    assert repr(path) in lines[0][1]
