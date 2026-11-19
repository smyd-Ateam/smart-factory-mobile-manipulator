"""결과 기록: JSON 한 줄씩 (JSON Lines)."""

import json
import os


def append_jsonl(path, record):
    # 먼저 직렬화: 실패해도 파일을 열지 않아 반쯤 쓴 줄이 남지 않음
    line = json.dumps(record, ensure_ascii=False) + '\n'
    path = os.path.expanduser(path)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'a', encoding='utf-8') as f:
        f.write(line)


def record_text(record):
    """로그에 남길 기록 내용: JSON 한 줄, JSON으로 못 바꾸면 repr."""
    try:
        return json.dumps(record, ensure_ascii=False)
    except Exception:
        return repr(record)


def writable_problem(path):
    """append_jsonl로 path에 쓸 수 있어 보이면 None, 아니면 이유 (노드가 뜰 때 미리 확인용).

    없는 폴더는 append_jsonl처럼 만든다. 파일은 만들지 않고, 기존 기록도 건드리지 않는다.
    """
    if not path:
        return 'empty path'
    path = os.path.expanduser(path)
    directory = os.path.dirname(path) or '.'
    try:
        os.makedirs(directory, exist_ok=True)
    except OSError as e:
        return str(e)
    if os.path.isdir(path):
        return 'path is a directory'
    if os.path.exists(path):
        if not os.access(path, os.W_OK):
            return 'file is not writable'
    elif not os.access(directory, os.W_OK | os.X_OK):
        return 'directory is not writable'
    return None
