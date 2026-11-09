import json

from mm_task_manager.result_log import append_jsonl


def test_append_jsonl_one_line_per_record(tmp_path):
    path = tmp_path / 'sub' / 'results.jsonl'      # 없는 폴더도 만든다
    append_jsonl(str(path), {'final_state': 'FAILED', 'detail': '도착 확인'})
    append_jsonl(str(path), {'final_state': 'SUCCEEDED', 'error_code': ''})
    lines = path.read_text(encoding='utf-8').splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0]) == {'final_state': 'FAILED', 'detail': '도착 확인'}
    assert json.loads(lines[1])['final_state'] == 'SUCCEEDED'
    assert '도착 확인' in lines[0]                 # 한글을 \u 이스케이프하지 않음


def test_append_jsonl_expands_home(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    append_jsonl('~/logs/results.jsonl', {'a': 1})
    assert (tmp_path / 'logs' / 'results.jsonl').exists()
