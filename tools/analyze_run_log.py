#!/usr/bin/env python3
"""Parse a Sunward JLOG/ELOG/RLOG UART dump into CSV, JSON and an SVG report."""
from __future__ import annotations

from pathlib import Path
import argparse
import csv
import html
import json
import math
import re
from typing import List, Optional


PREFIXES = ('RLOG BEGIN', 'JLOG,', 'ELOG,', 'RLOG,', 'RLOG END')
EVENT_NAMES = {1: 'BEGIN', 2: 'PHASE', 3: 'CHECKPOINT', 4: 'LOSS',
               5: 'REACQUIRE', 6: 'STOP'}


def payload(line: str) -> Optional[str]:
    positions = [line.find(prefix) for prefix in PREFIXES if line.find(prefix) >= 0]
    return line[min(positions):].strip() if positions else None


def parse_begin(line: str) -> dict[str, str]:
    return dict(re.findall(r'([A-Z]+)=([^\s]+)', line))


def integer(value: str, base: int = 10) -> int:
    return int(value.strip(), base)


def elapsed_ms(now: int, start: int) -> int:
    return (now - start) & 0xffffffff


def parse_dump(text: str) -> dict:
    blocks: List[dict] = []
    current: Optional[dict] = None
    for original in text.splitlines():
        line = payload(original)
        if line is None:
            continue
        if line.startswith('RLOG BEGIN'):
            current = {'metadata': parse_begin(line), 'journey': [], 'events': [],
                       'high_rate': [], 'complete': False}
            blocks.append(current)
            continue
        if current is None:
            continue
        if line == 'RLOG END':
            current['complete'] = True
            current = None
            continue
        fields = next(csv.reader([line]))
        if fields[0] == 'JLOG' and len(fields) == 10 and fields[1] != 'MS':
            meta, confidence = integer(fields[8], 16), integer(fields[9], 16)
            current['journey'].append({
                'ms': integer(fields[1]), 'progress_mm': integer(fields[2]),
                'yaw_deg': integer(fields[3]) / 10.0,
                'speed_left_mmps': integer(fields[4]),
                'speed_right_mmps': integer(fields[5]),
                'line_error': integer(fields[6]) / 2.0,
                'phase': integer(fields[7]), 'leg': meta & 0x03,
                'checkpoint': (meta >> 2) & 0x07,
                'line_valid': bool(meta & 0x20), 'line_lost': bool(meta & 0x40),
                'seek': bool(meta & 0x80),
                'line_confidence': (confidence & 0x0f) / 15.0,
                'heading_confidence': ((confidence >> 4) & 0x0f) / 15.0,
            })
        elif fields[0] == 'ELOG' and len(fields) == 17 and fields[1] != 'EV':
            confidence = integer(fields[12], 16)
            event = integer(fields[1])
            current['events'].append({
                'event': event, 'event_name': EVENT_NAMES.get(event, f'UNKNOWN_{event}'),
                'checkpoint': integer(fields[2]), 'ms': integer(fields[3]),
                'phase': integer(fields[4]), 'leg': integer(fields[5]),
                'raw_bits': integer(fields[6], 16),
                'line_error': integer(fields[7]) / 2.0,
                'yaw_deg': integer(fields[8]) / 10.0,
                'expected_yaw_deg': integer(fields[9]) / 10.0,
                'encoder_turn_deg': integer(fields[10]) / 10.0,
                'heading_error_deg': integer(fields[11]) / 2.0,
                'line_confidence': (confidence & 0x0f) / 15.0,
                'heading_confidence': ((confidence >> 4) & 0x0f) / 15.0,
                'target_left_mmps': integer(fields[13]),
                'target_right_mmps': integer(fields[14]),
                'speed_left_mmps': integer(fields[15]),
                'speed_right_mmps': integer(fields[16]),
            })
        elif fields[0] == 'RLOG' and len(fields) == 16 and fields[1] != 'SEQ':
            flags = integer(fields[15], 16)
            current['high_rate'].append({
                'sequence': integer(fields[1]), 'ms': integer(fields[2]),
                'state': integer(fields[3]), 'motion': integer(fields[4]),
                'phase': integer(fields[5]), 'leg': integer(fields[6]),
                'arc': integer(fields[7]), 'raw_bits': integer(fields[8], 16),
                'line_error': integer(fields[9]) / 2.0,
                'target_left_mmps': integer(fields[10]),
                'target_right_mmps': integer(fields[11]),
                'speed_left_mmps': integer(fields[12]),
                'speed_right_mmps': integer(fields[13]),
                'yaw_deg': integer(fields[14]) / 10.0,
                'flags': flags, 'line_valid': bool(flags & 0x01),
                'line_lost': bool(flags & 0x02), 'seek': bool(flags & 0x04),
                'zero_target': bool(flags & 0x08), 'zero_speed': bool(flags & 0x10),
                'fault': bool(flags & 0x20),
            })
    complete = [block for block in blocks if block['complete']]
    if not complete:
        raise ValueError('no complete RLOG BEGIN ... RLOG END block found')
    block = complete[-1]
    times = [row['ms'] for name in ('journey', 'events', 'high_rate') for row in block[name]]
    if not times:
        raise ValueError('complete log block contains no records')
    start = block['journey'][0]['ms'] if block['journey'] else times[0]
    for name in ('journey', 'events', 'high_rate'):
        for row in block[name]:
            row['time_s'] = elapsed_ms(row['ms'], start) / 1000.0
    return block


def write_csv(path: Path, rows: List[dict]) -> None:
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def finite_values(rows: List[dict], key: str) -> List[float]:
    return [float(row[key]) for row in rows if key in row and math.isfinite(float(row[key]))]


def summary(block: dict) -> dict:
    journey, events, high = block['journey'], block['events'], block['high_rate']
    speeds = finite_values(journey, 'speed_left_mmps') + finite_values(journey, 'speed_right_mmps')
    errors = finite_values(journey, 'line_error')
    headings = finite_values(journey, 'heading_confidence')
    lines = finite_values(journey, 'line_confidence')
    duration = max((row['time_s'] for group in (journey, events, high) for row in group), default=0.0)
    event_counts = {name: sum(row['event_name'] == name for row in events)
                    for name in EVENT_NAMES.values()}
    actual_counts = {'journey': len(journey), 'events': len(events), 'high_rate': len(high)}
    advertised = {
        'journey': int(block['metadata'].get('J', len(journey))),
        'events': int(block['metadata'].get('E', len(events))),
        'high_rate': int(block['metadata'].get('H', len(high))),
    }
    heading_errors = finite_values(events, 'heading_error_deg')
    warnings = []
    if actual_counts != advertised:
        warnings.append('record count differs from RLOG BEGIN; UART text may have dropped lines')
    if block['metadata'].get('GIT') in (None, 'LOCAL'):
        warnings.append('firmware Git fingerprint is unavailable; use build_firmware.py output')
    if block['metadata'].get('F') == '1':
        warnings.append('firmware froze this log because the run ended abnormally')
    return {
        'metadata': block['metadata'], 'complete': block['complete'],
        'duration_s': round(duration, 3),
        'record_counts': actual_counts, 'advertised_record_counts': advertised,
        'record_counts_match': actual_counts == advertised,
        'event_counts': event_counts,
        'checkpoint_events': [{key: row[key] for key in
            ('time_s', 'checkpoint', 'heading_error_deg', 'encoder_turn_deg',
             'line_confidence', 'heading_confidence')}
            for row in events if row['event_name'] == 'CHECKPOINT'],
        'max_abs_speed_mmps': max((abs(value) for value in speeds), default=0.0),
        'max_abs_line_error': max((abs(value) for value in errors), default=0.0),
        'max_abs_event_heading_error_deg': max((abs(value) for value in heading_errors), default=0.0),
        'min_line_confidence': min(lines, default=None),
        'min_heading_confidence': min(headings, default=None),
        'high_rate_zero_speed_samples': sum(bool(row['zero_speed']) for row in high),
        'high_rate_fault_samples': sum(bool(row['fault']) for row in high),
        'warnings': warnings,
    }


def svg_report(path: Path, block: dict, report: dict) -> None:
    rows = block['journey']
    width, height, left, right = 1200, 1000, 75, 30
    top, panel_h, gap = 75, 150, 30
    plot_w = width - left - right
    duration = max(report['duration_s'], 0.001)
    panels = [
        ('Segment progress (mm)', [('progress_mm', '#1565c0')]),
        ('Yaw (deg)', [('yaw_deg', '#6a1b9a')]),
        ('Wheel speed (mm/s)', [('speed_left_mmps', '#ef6c00'), ('speed_right_mmps', '#00897b')]),
        ('Line error', [('line_error', '#c62828')]),
        ('Confidence', [('line_confidence', '#2e7d32'), ('heading_confidence', '#283593')]),
    ]

    def x_of(seconds: float) -> float:
        return left + seconds / duration * plot_w

    chunks = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
              '<rect width="100%" height="100%" fill="white"/>',
              '<style>text{font-family:monospace;font-size:12px}.title{font-size:18px;font-weight:bold}.axis{stroke:#777;stroke-width:1}.grid{stroke:#ddd;stroke-width:1}.event{stroke:#999;stroke-dasharray:3 3}</style>',
              f'<text class="title" x="{left}" y="28">Sunward {html.escape(block["metadata"].get("VER", "?"))} run log</text>',
              f'<text x="{left}" y="49">GIT={html.escape(block["metadata"].get("GIT", "?"))} CFG={html.escape(block["metadata"].get("CFG", "?"))} MODE={html.escape(block["metadata"].get("MODE", "?"))} duration={duration:.3f}s</text>']
    for panel_index, (title, series) in enumerate(panels):
        y0 = top + panel_index * (panel_h + gap)
        keys = [key for key, _ in series]
        values = [float(row[key]) for row in rows for key in keys]
        low, high = (min(values), max(values)) if values else (0.0, 1.0)
        if title == 'Confidence': low, high = 0.0, 1.0
        if abs(high - low) < 1e-6: low -= 1.0; high += 1.0
        margin = 0.08 * (high - low)
        if title != 'Confidence': low -= margin; high += margin

        def y_of(value: float) -> float:
            return y0 + panel_h - (value - low) / (high - low) * panel_h

        chunks += [f'<text x="8" y="{y0 + 14}">{html.escape(title)}</text>',
                   f'<line class="axis" x1="{left}" y1="{y0}" x2="{left}" y2="{y0 + panel_h}"/>',
                   f'<line class="axis" x1="{left}" y1="{y0 + panel_h}" x2="{left + plot_w}" y2="{y0 + panel_h}"/>',
                   f'<text x="8" y="{y0 + 30}">{high:.2f}</text>',
                   f'<text x="8" y="{y0 + panel_h}">{low:.2f}</text>']
        for tick in range(1, 5):
            x = left + plot_w * tick / 5
            chunks.append(f'<line class="grid" x1="{x:.1f}" y1="{y0}" x2="{x:.1f}" y2="{y0 + panel_h}"/>')
        for event in block['events']:
            x = x_of(event['time_s'])
            chunks.append(f'<line class="event" x1="{x:.1f}" y1="{y0}" x2="{x:.1f}" y2="{y0 + panel_h}"/>')
        for key, color in series:
            points = ' '.join(f'{x_of(row["time_s"]):.1f},{y_of(float(row[key])):.1f}' for row in rows)
            chunks.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.5" points="{points}"/>')
        legend = '  '.join(f'{key}={color}' for key, color in series)
        chunks.append(f'<text x="{left + 8}" y="{y0 + 14}">{html.escape(legend)}</text>')
    chunks.append(f'<text x="{left}" y="{height - 12}">time 0..{duration:.3f}s; dashed lines are ELOG events</text>')
    chunks.append('</svg>')
    path.write_text('\n'.join(chunks) + '\n', encoding='utf-8')


def analyze(input_path: Path, output_dir: Path) -> dict:
    block = parse_dump(input_path.read_text(encoding='utf-8', errors='replace'))
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / 'journey.csv', block['journey'])
    write_csv(output_dir / 'events.csv', block['events'])
    write_csv(output_dir / 'high_rate.csv', block['high_rate'])
    report = summary(block)
    (output_dir / 'summary.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    svg_report(output_dir / 'report.svg', block, report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='UART text containing a complete log dump')
    parser.add_argument('-o', '--output-dir', type=Path,
                        help='output folder (default: <input stem>_analysis)')
    args = parser.parse_args()
    destination = args.output_dir or args.input.with_name(args.input.stem + '_analysis')
    try:
        report = analyze(args.input, destination)
    except (OSError, ValueError, csv.Error) as error:
        raise SystemExit('log analysis failed: ' + str(error)) from error
    print(f'PASS: {report["record_counts"]} duration={report["duration_s"]:.3f}s')
    print('Output: ' + str(destination.resolve()))


if __name__ == '__main__':
    main()
