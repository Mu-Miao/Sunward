#!/usr/bin/env python3
"""Exercise timeout/restart and complete-frame acceptance over a pseudo terminal."""
from pathlib import Path
import os
import pty
import select
import subprocess
import sys
import tempfile
import threading


root = Path(__file__).resolve().parents[2]
capture_tool = root / 'tools/capture_run_log.py'
master, slave = pty.openpty()
port = os.ttyname(slave)
stop = threading.Event()
requests = []


def fake_car() -> None:
    pending = bytearray()
    while not stop.is_set():
        ready, _, _ = select.select([master], [], [], 0.1)
        if not ready:
            continue
        pending.extend(os.read(master, 1024))
        while b'\n' in pending:
            raw, _, pending = pending.partition(b'\n')
            command = raw.strip()
            if command != b'LOG!':
                continue
            requests.append(command)
            if len(requests) == 1:
                os.write(master, b'noise\r\nRLOG BEGIN VER=TEST J=0 E=0 H=0\r\n')
            else:
                os.write(master, b'noise\r\nRLOG BEGIN VER=TEST J=0 E=0 H=0\r\nRLOG END\r\n')


worker = threading.Thread(target=fake_car, daemon=True)
worker.start()
try:
    with tempfile.TemporaryDirectory(prefix='sunward-log-capture-') as folder:
        output = Path(folder) / 'capture.txt'
        result = subprocess.run(
            [sys.executable, str(capture_tool), port, '--timeout', '0.3',
             '--retries', '2', '--no-analyze', '-o', str(output)],
            capture_output=True, text=True, timeout=5)
        assert result.returncode == 0, result.stdout + result.stderr
        assert len(requests) == 2
        assert output.read_bytes().startswith(b'RLOG BEGIN')
        assert output.read_bytes().rstrip().endswith(b'RLOG END')
finally:
    stop.set()
    worker.join(timeout=1)
    os.close(master)
    os.close(slave)

print('PASS: serial capture retries an incomplete wireless dump and saves only a complete block.')
