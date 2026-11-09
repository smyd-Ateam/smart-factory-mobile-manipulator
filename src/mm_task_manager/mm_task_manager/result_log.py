"""결과 기록: JSON 한 줄씩 (JSON Lines)."""

import json
import os


def append_jsonl(path, record):
    path = os.path.expanduser(path)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'a', encoding='utf-8') as f:
        f.write(json.dumps(record, ensure_ascii=False) + '\n')
