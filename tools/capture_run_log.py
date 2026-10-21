#!/usr/bin/env python3
"""Capture a complete Sunward log over a local/remote debugger serial port."""
from pathlib import Path
import argparse
import os
import select
import subprocess
import sys
import termios
import time


BAUDS = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def configure(fd: int, baud: int) -> None:
    if baud not in BAUDS:
        raise ValueError(f'unsupported baud rate: {baud}')
    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNPAR
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = BAUDS[baud]
    attrs[5] = BAUDS[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def write_all(fd: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        try:
            written = os.write(fd, view)
            view = view[written:]
        except BlockingIOError:
            select.select([], [fd], [], 1.0)


def capture_attempt(fd: int, timeout: float) -> bytes | None:
    write_all(fd, b'LOGSTOP!\r\n')
    time.sleep(0.1)
    termios.tcflush(fd, termios.TCIFLUSH)
    write_all(fd, b'LOG!\r\n')
    received = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        ready, _, _ = select.select([fd], [], [], min(0.5, remaining))
        if not ready:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        received.extend(chunk)
        begin = received.find(b'RLOG BEGIN')
        end = received.find(b'RLOG END', begin if begin >= 0 else 0)
        if begin >= 0 and end >= 0:
            line_end = received.find(b'\n', end)
            if line_end < 0:
                line_end = end + len(b'RLOG END')
            else:
                line_end += 1
            return bytes(received[begin:line_end])
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', help='serial device, for example /dev/ttyACM0')
    parser.add_argument('-b', '--baud', type=int, default=115200)
    parser.add_argument('-o', '--output', type=Path, default=Path('capture.txt'))
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='seconds allowed for each complete dump')
    parser.add_argument('--retries', type=int, default=3)
    parser.add_argument('--no-analyze', action='store_true')
    args = parser.parse_args()
    if args.timeout <= 0 or args.retries < 1:
        parser.error('timeout and retries must be positive')

    flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
    try:
        fd = os.open(args.port, flags)
        configure(fd, args.baud)
    except (OSError, ValueError) as error:
        raise SystemExit(f'cannot open serial port {args.port}: {error}') from error

    block = None
    try:
        for attempt in range(1, args.retries + 1):
            print(f'capture attempt {attempt}/{args.retries} ...', flush=True)
            block = capture_attempt(fd, args.timeout)
            if block is not None:
                break
            print('incomplete dump; restarting from RLOG BEGIN', file=sys.stderr)
    finally:
        os.close(fd)
    if block is None:
        raise SystemExit('capture failed: no complete RLOG BEGIN/RLOG END block')

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(block)
    print('saved: ' + str(args.output.resolve()))
    if not args.no_analyze:
        analyzer = Path(__file__).with_name('analyze_run_log.py')
        subprocess.run([sys.executable, str(analyzer), str(args.output)], check=True)


if __name__ == '__main__':
    main()
