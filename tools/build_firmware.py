#!/usr/bin/env python3
"""Build all firmware units with GCC; no flashing or hardware access."""
from pathlib import Path
import argparse
import hashlib
import json
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [
    "Core/Src/adc.c",
    "Core/Src/car_app.c",
    "Core/Src/car_button.c",
    "Core/Src/car_command.c",
    "Core/Src/car_heading.c",
    "Core/Src/car_line.c",
    "Core/Src/car_pi.c",
    "Core/Src/car_power.c",
    "Core/Src/car_route.c",
    "Core/Src/car_signal.c",
    "Core/Src/car_task3.c",
    "Core/Src/car_task3_port.c",
    "Core/Src/car_task4.c",
    "Core/Src/car_ui.c",
    "Core/Src/car_wheel.c",
    "Core/Src/dual_pi_test.c",
    "Core/Src/gpio.c",
    "Core/Src/lab_uart_async.c",
    "Core/Src/left_pi_test.c",
    "Core/Src/main.c",
    "Core/Src/mpu6050.c",
    "Core/Src/right_p_test.c",
    "Core/Src/soft_i2c.c",
    "Core/Src/stm32f1xx_hal_msp.c",
    "Core/Src/stm32f1xx_it.c",
    "Core/Src/syscalls.c",
    "Core/Src/sysmem.c",
    "Core/Src/system_stm32f1xx.c",
    "Core/Src/task_oled.c",
    "Core/Src/tim.c",
    "Core/Src/usart.c",
    "Core/Src/wheel_speed_diag.c",
    "Core/Src/yaw_diag.c",
    "Core/Startup/startup_stm32f103rctx.s",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c"
]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', help='Path to arm-none-eabi-gcc')
    args = parser.parse_args()
    candidates = sorted(Path('/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins').glob(
        'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/arm-none-eabi-gcc'))
    compiler = args.cc or (str(candidates[-1]) if candidates else None) or shutil.which('arm-none-eabi-gcc')
    if not compiler:
        parser.error('arm-none-eabi-gcc not found; provide --cc')
    compiler = Path(compiler).resolve()
    out = ROOT / 'build/Debug'
    out.mkdir(parents=True, exist_ok=True)
    common = ['-ICore/Inc', '-IDrivers/STM32F1xx_HAL_Driver/Inc',
        '-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy', '-IDrivers/CMSIS/Device/ST/STM32F1xx/Include',
        '-IDrivers/CMSIS/Include', '-mcpu=cortex-m3', '-mthumb', '-g',
        '--specs=nano.specs', '--specs=nosys.specs', '-DDEBUG']
    objects, commands = [], []
    with (out / 'arm_build.log').open('w') as log:
        for source in SOURCES:
            obj = 'build/Debug/.obj/' + str(Path(source).with_suffix('.o'))
            (ROOT / obj).parent.mkdir(parents=True, exist_ok=True)
            flags = ['-x', 'assembler-with-cpp'] if source.endswith('.s') else [
                '-xc', '-std=c11', '-DUSE_HAL_DRIVER', '-DSTM32F103xE', '-Og', '-Wall',
                '-ffunction-sections', '-fdata-sections', '-funsigned-char']
            command = [str(compiler), '-c'] + flags + common + ['-o', obj, '-MMD', source]
            print('BUILD ' + source, file=log, flush=True)
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=log, check=True)
            objects.append(obj)
            commands.append({'directory': str(ROOT), 'file': str(ROOT/source), 'command': shlex.join(command)})
        (out/'compile_commands.json').write_text(json.dumps(commands, indent=2)+'\n')
        link = [str(compiler), '-mcpu=cortex-m3', '-mthumb', '-T', 'STM32F103RCTX_FLASH.ld',
            '-Wl,--gc-sections', '-Wl,--print-memory-usage', '-Wl,-Map=build/Debug/Sunward.map',
            '--specs=nano.specs', '--specs=nosys.specs', '-o', 'build/Debug/Sunward.elf'] + objects + ['-lm']
        subprocess.run(link, cwd=ROOT, stdout=log, stderr=log, check=True)
        subprocess.run([str(compiler.parent/'arm-none-eabi-objcopy'), '-O', 'ihex',
            str(out/'Sunward.elf'), str(out/'Sunward.hex')], cwd=ROOT, check=True)
        size = subprocess.check_output([str(compiler.parent/'arm-none-eabi-size'), str(out/'Sunward.elf')], text=True)
        print(size, file=log)
    rows = (out/'Sunward.hex').read_text().splitlines()
    memory, base, eof = {}, 0, 0
    for n, row in enumerate(rows):
        assert row.startswith(':')
        b = bytes.fromhex(row[1:])
        assert len(b)==b[0]+5 and sum(b)%256==0
        kind, address, payload = b[3], int.from_bytes(b[1:3], 'big'), b[4:-1]
        if kind==4: base=int.from_bytes(payload, 'big')<<16
        elif kind==0:
            for offset, value in enumerate(payload): memory[base+address+offset]=value
        elif kind==1: eof+=1; assert n==len(rows)-1
    assert eof==1 and min(memory)==0x08000000 and max(memory)<0x08040000
    image=bytes(memory.get(a,255) for a in range(min(memory),max(memory)+1))
    assert b'TASK 4 V19 EDGE' in image and b'TASK 3 V3 GYRO' in image
    assert b'TASK 2 V22 T4LINE' in image and b'FW=T2V22' in image
    assert 0x20000000 < int.from_bytes(image[:4], 'little') <= 0x2000C000
    reset=int.from_bytes(image[4:8], 'little')
    assert reset & 1 and 0x08000000 <= (reset & ~1) < 0x08040000
    manifest={'base_branch':'codex/task4-v14-rolling',
        'base_commit':'f2802b6bf4152c0142424c0d71640e7091490cd5',
        'task4_title':'TASK 4 V19 EDGE', 'units_compiled':len(SOURCES),
        'diagonal_mmps':270, 'arc_mmps':240, 'entry_mmps':50, 'task4_max_cmd':2200,
        'task2_title':'TASK 2 V22 T4LINE', 'task2_fw_tag':'T2V22',
        'task2_straight_mmps':270, 'task2_arc_mmps':240, 'task2_max_cmd':2200,
        'task2_edge_follow_mmps':70, 'task2_edge_follow_diff_mmps':50,
        'task2_edge_follow_entry':'capture-at-arc-entry', 'task2_edge_follow_exit_ms':60,
        'task2_edge_follow_min_ms':200, 'task2_edge_follow_capture_mmps':50,
        'task2_edge_follow_seek_mmps':40, 'task2_edge_follow_white_ms':500,
        'task2_edge_follow_white_mm':20, 'task2_edge_follow_white_deg':12,
        'task2_edge_follow_gap_mm':35, 'task2_edge_follow_gap_deg':18,
        'task2_edge_follow_recovery_ms':30, 'task2_edge_follow_accel_mmps2':100,
        'task2_edge_follow_bounds':'120mm/2500ms/45deg',
        'preview':'first valid C/D candidate inside original window; 30ms point confirmation',
        'edge_follow':True, 'edge_follow_mmps':70, 'edge_follow_diff_mmps':50,
        'edge_follow_mm':120, 'edge_follow_ms':2500, 'edge_follow_deg':45,
        'white_seek_ms':500, 'continuous_motion':True, 'turn_inner_mmps':20, 'turn_outer_mmps':100,
        'turn_limit_mm':120, 'turn_limit_ms':4000,
        'hex_checksum_eof_titles_vectors_valid':True, 'size':size,
        'field_verified':False, 'flashed':False,
        'sha256':{n:hashlib.sha256((out/n).read_bytes()).hexdigest() for n in ['Sunward.elf','Sunward.hex']},
        'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in
            SOURCES + sorted(str(p.relative_to(ROOT)) for p in (ROOT/'Core/Inc').glob('*.h')) + ['STM32F103RCTX_FLASH.ld']}}
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(size.strip())
    print('PASS: all ARM units linked; HEX checksums, EOF, version titles, vectors verified. No flashing.')

if __name__=='__main__':
    main()
