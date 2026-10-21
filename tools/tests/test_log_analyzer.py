#!/usr/bin/env python3
from pathlib import Path
import csv
import json
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


root = Path(__file__).resolve().parents[2]
sample = """boot noise
UART0 RLOG BEGIN VER=ARC12 GIT=12345678 CFG=A1B2C3D4 MODE=RTOS J=2 E=2 H=2 F=1 WHY=16 RAM=17788B
JLOG,MS,PROG,Y10,VL,VR,E2,PH,META,CONF
JLOG,4294967200,0,386,100,102,0,5,21,FF
JLOG,104,40,443,110,120,2,5,E5,CA
ELOG,EV,CP,MS,PH,LEG,RAW,E2,Y10,EY10,ET10,HE2,CONF,TL,TR,VL,VR
ELOG,1,0,4294967200,5,1,09,0,386,386,0,0,FF,100,100,98,99
ELOG,4,1,104,5,1,08,2,443,450,445,1,CA,90,130,85,100
RLOG,SEQ,MS,ST,M,PH,LEG,ARC,RAW,E2,TL,TR,VL,VR,Y10,FLAGS
RLOG,7,54,1,12,5,1,1,09,0,100,120,95,115,420,01
RLOG,8,104,16,12,5,1,1,08,2,0,0,0,0,443,32
RLOG END
tail noise
"""

with tempfile.TemporaryDirectory(prefix='sunward-log-test-') as folder:
    folder = Path(folder)
    source, output = folder / 'capture.txt', folder / 'analysis'
    source.write_text(sample, encoding='utf-8')
    result = subprocess.run([sys.executable, str(root / 'tools/analyze_run_log.py'),
                             str(source), '-o', str(output)],
                            capture_output=True, text=True, check=True)
    assert 'PASS:' in result.stdout
    report = json.loads((output / 'summary.json').read_text(encoding='utf-8'))
    assert report['metadata']['VER'] == 'ARC12'
    assert report['metadata']['CFG'] == 'A1B2C3D4'
    assert report['record_counts'] == {'journey': 2, 'events': 2, 'high_rate': 2}
    assert report['record_counts_match']
    assert abs(report['duration_s'] - 0.2) < 1e-6
    assert report['event_counts']['LOSS'] == 1
    assert 'ended abnormally' in report['warnings'][0]
    assert report['high_rate_zero_speed_samples'] == 1
    with (output / 'journey.csv').open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert rows[1]['leg'] == '1' and rows[1]['checkpoint'] == '1'
    assert rows[1]['line_lost'] == 'True' and rows[1]['seek'] == 'True'
    assert abs(float(rows[1]['time_s']) - 0.2) < 1e-6
    svg = (output / 'report.svg').read_text(encoding='utf-8')
    assert '<svg' in svg and 'Sunward ARC12 run log' in svg
    ET.parse(output / 'report.svg')

    incomplete = folder / 'incomplete.txt'
    incomplete.write_text(sample.replace('RLOG END', ''), encoding='utf-8')
    failed = subprocess.run([sys.executable, str(root / 'tools/analyze_run_log.py'),
                             str(incomplete)], capture_output=True, text=True)
    assert failed.returncode != 0 and 'no complete' in failed.stderr

print('PASS: log parser normalizes wrapped timestamps and emits CSV, summary JSON and dependency-free SVG.')
