#!/usr/bin/env python3
"""Build separate production modules with mock HAL; never access hardware."""
from pathlib import Path
import argparse
import os
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--sanitize', action='store_true', help='Enable ASan and UBSan')
args = parser.parse_args()
root = args.project.resolve()
tests = Path(__file__).resolve().parent
modules = sorted(p for p in (root / 'Core/Src').glob('car_*.c')
                 if p.name != 'car_rtos.c')
assert len(modules) == 17, 'Expected all 17 host-testable car modules in Core/Src'
assert not any('#include' in line and '.c"' in line
               for path in modules for line in path.read_text().splitlines())
flags = [os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
         '-Wno-misleading-indentation',
         '-DCAR_APP_TEST', '-I' + str(tests), '-I' + str(root / 'Core/Inc')]
if args.sanitize:
    flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
with tempfile.TemporaryDirectory(prefix='sunward-host-tests-') as folder:
    hybrid = str(Path(folder) / 'hybrid')
    subprocess.run(flags + [str(tests / 'test_hybrid.c'),
                            str(root / 'Core/Src/car_hybrid.c'),
                            '-lm', '-o', hybrid], check=True)
    result = subprocess.run([hybrid], capture_output=True, text=True, check=True)
    print('hybrid: ' + result.stdout.strip())
    boundary = str(Path(folder) / 'rtos_boundary')
    subprocess.run(flags + [str(tests / 'test_rtos_boundary.c'),
                            str(root / 'Core/Src/car_tick.c'),
                            '-o', boundary], check=True)
    result = subprocess.run([boundary], capture_output=True, text=True, check=True)
    print('rtos_boundary: ' + result.stdout.strip())
    run_log = str(Path(folder) / 'run_log')
    subprocess.run(flags + [str(tests / 'test_run_log.c'),
                            str(root / 'Core/Src/car_run_log.c'),
                            '-lm', '-o', run_log], check=True)
    result = subprocess.run([run_log], capture_output=True, text=True, check=True)
    print('run_log: ' + result.stdout.strip())
    for suite in ['motion', 'task2', 'task3', 'task4', 'seek', 'turn', 'heading']:
        for public in [False, True]:
            name = suite + ('_car_app' if public else '_legacy_api')
            binary = str(Path(folder) / name)
            sources = [str(tests / ('test_' + suite + '.c'))] + [str(p) for p in modules]
            aliases = []
            if public:
                aliases = ['-DDualPITest_' + n + '=CarApp_' + n
                           for n in ['Begin', 'EnableBluetooth', 'EnableHeading', 'Poll']]
            else:
                sources.append(str(root / 'Core/Src/dual_pi_test.c'))
            subprocess.run(flags + aliases + sources + ['-lm', '-o', binary], check=True)
            result = subprocess.run([binary], capture_output=True, text=True)
            if result.returncode:
                print(result.stdout + result.stderr, flush=True)
                result.check_returncode()
            print(name + ': ' + result.stdout.strip())
print('PASS: hybrid, SysTick and run-log units; motion/task2/task3/task4/seek/turn/heading regressions; CarApp and compatibility APIs; separate module linkage.')
