import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from robot_telemetry.telemetry_csv import CsvLog, JsonlLog  # noqa: E402


def test_csvlog_writes_header_and_rows(tmp_path):
    path = str(tmp_path / 'test.csv')
    log = CsvLog(path, ['sec', 'value'])
    assert log.count == 0
    log.write([1, 2.5])
    log.write([2, 3.5])
    assert log.count == 2
    log.close()

    with open(path) as fh:
        lines = fh.read().splitlines()
    assert lines == ['sec,value', '1,2.5', '2,3.5']


def test_jsonllog_appends_json_lines(tmp_path):
    path = str(tmp_path / 'test.jsonl')
    log = JsonlLog(path)
    log.write({'topic': '/odom', 'sec': 1, 'data': {'x': 1.0}})
    log.write({'topic': '/scan', 'sec': 2, 'data': {'min': 0.8}})
    log.close()

    with open(path) as fh:
        events = [json.loads(line) for line in fh]
    assert events[0]['topic'] == '/odom'
    assert events[1]['data']['min'] == 0.8
    assert len(events) == 2
