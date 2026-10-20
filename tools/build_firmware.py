#!/usr/bin/env python3
"""Build the STM32F103 firmware, including the pinned FreeRTOS kernel."""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=Path('build/Sunward.elf'))
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]

compiler = shutil.which('arm-none-eabi-gcc')
if compiler is None:
    candidates = sorted(Path('/opt/st').glob(
        '**/gnu-tools-for-stm32*/tools/bin/arm-none-eabi-gcc'))
    if not candidates:
        raise SystemExit('arm-none-eabi-gcc not found')
    compiler = str(candidates[-1])

kernel = root / 'Middlewares/Third_Party/FreeRTOS-Kernel'
if not (kernel / 'tasks.c').is_file():
    raise SystemExit('FreeRTOS submodule missing; run: git submodule update --init')

includes = [
    root / 'Core/Inc',
    root / 'Drivers/STM32F1xx_HAL_Driver/Inc',
    root / 'Drivers/STM32F1xx_HAL_Driver/Inc/Legacy',
    root / 'Drivers/CMSIS/Device/ST/STM32F1xx/Include',
    root / 'Drivers/CMSIS/Include',
    kernel / 'include',
    kernel / 'portable/GCC/ARM_CM3',
]
sources = sorted((root / 'Core/Src').glob('*.c'))
sources += sorted(p for p in (root / 'Drivers/STM32F1xx_HAL_Driver/Src').glob('*.c')
                  if not p.name.endswith('_template.c'))
sources += [
    kernel / 'list.c', kernel / 'queue.c', kernel / 'tasks.c',
    kernel / 'portable/GCC/ARM_CM3/port.c',
    kernel / 'portable/MemMang/heap_4.c',
    root / 'Core/Startup/startup_stm32f103rctx.s',
]

common = [
    '-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft', '-std=gnu11',
    '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
    '-Wno-unused-variable', '-ffunction-sections', '-fdata-sections',
    '-DUSE_HAL_DRIVER', '-DSTM32F103xE',
] + ['-I' + str(path) for path in includes]

output = args.output if args.output.is_absolute() else root / args.output
output.parent.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix='sunward-arm-') as folder:
    objects = []
    for index, source in enumerate(sources):
        obj = Path(folder) / f'{index:03d}_{source.stem}.o'
        subprocess.run([compiler, *common, '-c', str(source), '-o', str(obj)],
                       check=True)
        objects.append(str(obj))
    subprocess.run([
        compiler, '-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft',
        '-Wl,--gc-sections', '-T' + str(root / 'STM32F103RCTX_FLASH.ld'),
        *objects, '-specs=nano.specs', '-specs=nosys.specs', '-lm',
        '-o', str(output),
    ], check=True)

size = Path(compiler).with_name('arm-none-eabi-size')
subprocess.run([str(size), str(output)], check=True)
print('PASS: firmware linked at ' + str(output))
