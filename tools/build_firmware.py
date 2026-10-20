#!/usr/bin/env python3
"""Build and verify the RTOS and/or bare-metal STM32F103 firmware."""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--mode', choices=['all', 'rtos', 'baremetal'], default='all')
parser.add_argument('--output-dir', type=Path, default=Path('build'))
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
modes = ['rtos', 'baremetal'] if args.mode == 'all' else [args.mode]

compiler = shutil.which('arm-none-eabi-gcc')
if compiler is None:
    candidates = sorted(Path('/opt/st').glob(
        '**/gnu-tools-for-stm32*/tools/bin/arm-none-eabi-gcc'))
    if not candidates:
        raise SystemExit('arm-none-eabi-gcc not found')
    compiler = str(candidates[-1])

kernel = root / 'Middlewares/Third_Party/FreeRTOS-Kernel'
if 'rtos' in modes and not (kernel / 'tasks.c').is_file():
    raise SystemExit('FreeRTOS submodule missing; run: git submodule update --init')

base_includes = [
    root / 'Core/Inc',
    root / 'Drivers/STM32F1xx_HAL_Driver/Inc',
    root / 'Drivers/STM32F1xx_HAL_Driver/Inc/Legacy',
    root / 'Drivers/CMSIS/Device/ST/STM32F1xx/Include',
    root / 'Drivers/CMSIS/Include',
]
core_sources = sorted((root / 'Core/Src').glob('*.c'))
hal_sources = sorted(
    p for p in (root / 'Drivers/STM32F1xx_HAL_Driver/Src').glob('*.c')
    if not p.name.endswith('_template.c'))
kernel_sources = [
    kernel / 'list.c', kernel / 'queue.c', kernel / 'tasks.c',
    kernel / 'portable/GCC/ARM_CM3/port.c',
    kernel / 'portable/MemMang/heap_4.c',
]
startup = root / 'Core/Startup/startup_stm32f103rctx.s'
output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
output_dir.mkdir(parents=True, exist_ok=True)


def build(mode: str) -> Path:
    use_rtos = mode == 'rtos'
    includes = list(base_includes)
    sources = core_sources + hal_sources + [startup]
    if use_rtos:
        includes += [kernel / 'include', kernel / 'portable/GCC/ARM_CM3']
        sources += kernel_sources
    common = [
        '-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft', '-std=gnu11',
        '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
        '-Wno-unused-variable', '-ffunction-sections', '-fdata-sections',
        '-DUSE_HAL_DRIVER', '-DSTM32F103xE',
        '-DCAR_USE_FREERTOS=' + ('1' if use_rtos else '0'),
    ] + ['-I' + str(path) for path in includes]
    output = output_dir / ('Sunward-rtos.elf' if use_rtos else 'Sunward-baremetal.elf')
    with tempfile.TemporaryDirectory(prefix='sunward-arm-' + mode + '-') as folder:
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
    return output


size = Path(compiler).with_name('arm-none-eabi-size')
nm = Path(compiler).with_name('arm-none-eabi-nm')
objdump = Path(compiler).with_name('arm-none-eabi-objdump')
for selected in modes:
    elf = build(selected)
    size_result = subprocess.run([str(size), str(elf)], check=True,
                                 capture_output=True, text=True)
    print(size_result.stdout, end='')
    fields = size_result.stdout.splitlines()[1].split()
    text_bytes, data_bytes, bss_bytes = map(int, fields[:3])
    flash_bytes = text_bytes + data_bytes
    ram_bytes = data_bytes + bss_bytes
    if flash_bytes > 224 * 1024:
        raise SystemExit(selected + ': flash headroom below 32 KiB')
    if ram_bytes > 36 * 1024:
        raise SystemExit(selected + ': static RAM headroom below 12 KiB')
    print(f'PASS: {selected} resources FLASH={flash_bytes}/262144 '
          f'RAM={ram_bytes}/49152 bytes')
    symbols = subprocess.run([str(nm), str(elf)], check=True,
                             capture_output=True, text=True).stdout
    if ' T CarEmergency_Stop\n' not in symbols:
        raise SystemExit(selected + ': missing common emergency stop')
    if symbols.count(' T SysTick_Handler\n') != 1:
        raise SystemExit(selected + ': expected exactly one SysTick_Handler')
    disassembly = subprocess.run([str(objdump), '-d', str(elf)], check=True,
                                 capture_output=True, text=True).stdout
    for fatal in ['NMI_Handler', 'HardFault_Handler', 'MemManage_Handler',
                  'BusFault_Handler', 'UsageFault_Handler', 'Error_Handler']:
        fatal_body = disassembly.split('<' + fatal + '>:', 1)[1].split('\n\n', 1)[0]
        if 'CarEmergency_Stop' not in fatal_body:
            raise SystemExit(selected + ': ' + fatal + ' bypasses emergency stop')
        if fatal != 'Error_Handler' and ('push' in fatal_body or ' sp' in fatal_body):
            raise SystemExit(selected + ': ' + fatal + ' unexpectedly uses the faulted stack')
    marker = '<SysTick_Handler>:'
    handler = disassembly.split(marker, 1)[1].split('\n\n', 1)[0]
    if 'HAL_IncTick' not in handler:
        raise SystemExit(selected + ': SysTick no longer maintains HAL time')
    if selected == 'rtos':
        for symbol in [' T xPortSysTickHandler\n', ' T vApplicationTickHook\n',
                       ' T CarRTOS_FatalStop\n']:
            if symbol not in symbols:
                raise SystemExit('rtos: missing symbol ' + symbol.strip())
        for call in ['CarTick_UseKernel', 'xPortSysTickHandler']:
            if call not in handler:
                raise SystemExit('rtos: SysTick handoff missing ' + call)
        print('PASS: RTOS SysTick contains HAL-before-start and kernel-after-start paths')
    elif 'xPortSysTickHandler' in symbols:
        raise SystemExit('baremetal: FreeRTOS port unexpectedly linked')
    elif 'CarTick_UseKernel' in handler:
        raise SystemExit('baremetal: SysTick unexpectedly references the kernel')
    else:
        print('PASS: bare-metal SysTick contains only the HAL time-base path')
    print('PASS: ' + selected + ' firmware linked at ' + str(elf))
