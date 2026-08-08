#!/usr/bin/env python3
"""CSV and JSON-lines log helpers for the telemetry recorder.

Deliberately free of ROS imports so it can be unit-tested standalone.
"""

import csv
import json


class CsvLog:
    """Append-only CSV file; header row written once on open."""

    def __init__(self, path, header):
        self.path = path
        self.count = 0
        self._file = open(path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(list(header))
        self._file.flush()

    def write(self, row):
        self._writer.writerow(list(row))
        self._file.flush()
        self.count += 1

    def close(self):
        self._file.close()


class JsonlLog:
    """Append-only JSON-lines log (one JSON object per line)."""

    def __init__(self, path):
        self.path = path
        self.count = 0
        self._file = open(path, 'a')

    def write(self, event):
        self._file.write(json.dumps(event) + '\n')
        self._file.flush()
        self.count += 1

    def close(self):
        self._file.close()
