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
    assert b'TASK 1 V4 FAST' in image
    assert b'TASK 4 V27 SMTH' in image and b'TASK 3 V27 SMTH' in image
    assert b'TASK 2 V27 SMTH' in image and b'FW=T2V27' in image
    assert 0x20000000 < int.from_bytes(image[:4], 'little') <= 0x2000C000
    reset=int.from_bytes(image[4:8], 'little')
    assert reset & 1 and 0x08000000 <= (reset & ~1) < 0x08040000
    manifest={'base_branch':'codex/task234-smooth-v27',
        'base_commit':'b5e0a1b',
        'rollback_branch':'codex/v26-before-line-smoothing',
        'line_control':'filtered PD + normal differential slew + rising-base ramp; wheel PI unchanged',
        'line_kp':48, 'line_kd':0.40, 'line_d_tau_s':0.050, 'line_d_limit':12,
        'line_diff_rate_mmps2':1600, 'line_recovery_accel_mmps2':200,
        'line_integral_enabled':False, 'emergency_edge_and_loss_smoothing':False,
        'task1_title':'TASK 1 V4 FAST', 'task1_goal_mm':1000,
        'task1_cruise_mmps':400, 'task1_start_mmps':40, 'task1_accel_mmps2':300,
        'task1_slow_mm':300, 'task1_max_cmd':3600, 'task1_timeout_ms':13500,
        'task1_settle_ms':200, 'task1_brake_timeout_ms':1500,
        'task1_min_mmps':40, 'task1_signal_ms':500,
        'task3_title':'TASK 3 V27 SMTH', 'task3_laps':1, 'task3_timeout_ms':40000,
        'task3_profile':'shared V27 normal PD and Task4 EDGE; independent40s; no lap wrapper', 'task3_max_cmd':3600,
        'task4_title':'TASK 4 V27 SMTH', 'units_compiled':len(SOURCES),
        'diagonal_mmps':400, 'arc_mmps':300, 'diagonal_accel_mmps2':250, 'approach_mm':180, 'entry_mmps':50, 'task4_max_cmd':3600,
        'task2_title':'TASK 2 V27 SMTH', 'task2_fw_tag':'T2V27',
        'task2_straight_mmps':400, 'task2_arc_mmps':300, 'task2_accel_mmps2':250, 'task2_ab_approach':False, 'task2_approach_scope':'CD only', 'task2_approach_mm':160, 'task2_approach_min_mmps':160, 'task2_max_cmd':3600,
        'task2_arc_engine':'CarTask3_Update + CarTask3Port_Apply',
        'task2_cd_reference':'latched actual gyro heading at confirmed C',
        'task2_cd_accel_mmps2':250, 'task2_cd_start_mmps':50, 'task2_edge_mmps':120,
        'task2_online_gyro_bias_learning':False,
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
        'competition_speed_guard_mmps':500, 'other_modes_speed_guard_mmps':350,
        'measurement_reference':'2026-10-28 suspended-wheel PWM sweep: full7200 L1130/R1155mmps; not loaded ground speed',
        'field_verified':False, 'flashed':False,
        'sha256':{n:hashlib.sha256((out/n).read_bytes()).hexdigest() for n in ['Sunward.elf','Sunward.hex']},
        'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in
            SOURCES + sorted(str(p.relative_to(ROOT)) for p in (ROOT/'Core/Inc').glob('*.h')) + ['STM32F103RCTX_FLASH.ld']}}
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(size.strip())
    print('PASS: all ARM units linked; HEX checksums, EOF, version titles, vectors verified. No flashing.')

if __name__=='__main__':
    main()
