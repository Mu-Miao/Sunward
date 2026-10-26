from pathlib import Path
import re,json,hashlib,subprocess
r=Path.cwd();base=r/'deliverables/task1-v2-fast/validation/host_tests_asan_ubsan.log';old=base.read_text();new=Path('/tmp/all-speed-tests.log').read_text()
patterns={
 'task1':r'IDEAL Task1 target-following including coast/settle: (\d+) ms',
 'task2':r'IDEAL Task2 one lap: (\d+) ms',
 'task3':r'IDEAL Task3 one-lap continuous schedule: (\d+) ms',
 'task4':r'IDEAL continuous four-lap schedule: (\d+) ms'}
result={'baseline_commit':'2850bfb73ff34dd624e703744207831eccf885f1','simulation_only':True,'physical_results_claimed':False,'schedule_ms':{}}
for key,pat in patterns.items():
 before=[int(x) for x in re.findall(pat,old)];after=[int(x) for x in re.findall(pat,new)]
 assert len(before)==len(after)==2 and before[0]==before[1] and after[0]==after[1]
 assert after[0]<before[0],key
 result['schedule_ms'][key]={'before':before[0],'after':after[0],'reduction':before[0]-after[0]}
original=json.loads((r/'deliverables/task1-v2-fast/manifest.json').read_text())
current=json.loads((r/'build/Debug/manifest.json').read_text())
diffs=[f for f,h in original['source_sha256'].items() if h!=current['source_sha256'][f]]
assert set(diffs)=={'Core/Inc/car_config.h','Core/Src/car_task4.c','Core/Src/car_ui.c'},diffs
result['changed_firmware_source_files']=diffs
result['unchanged_control_sources']=['car_app.c','car_route.c','car_task3.c','car_task3_port.c','car_line.c','car_heading.c','car_pi.c','car_wheel.c']
for f,h in current['source_sha256'].items():assert hashlib.sha256((r/f).read_bytes()).hexdigest()==h,f
# Only normal speed, straight acceleration and Task1 approach macros may change.
a=subprocess.check_output(['git','show','2850bfb:Core/Inc/car_config.h'],text=True)
b=(r/'Core/Inc/car_config.h').read_text()
def macros(s):return {m.group(1):m.group(2).split('/*')[0].strip() for m in re.finditer(r'^#define (DPT_\w+) (.+)$',s,re.M)}
a,b=macros(a),macros(b)
changed={k:[a.get(k),b.get(k)] for k in a.keys()|b.keys() if a.get(k)!=b.get(k)}
assert set(changed)=={'DPT_ROUTE_CRUISE_MMPS','DPT_ROUTE_SLOW_MM','DPT_TASK2_STRAIGHT_MMPS','DPT_TASK2_ARC_MMPS','DPT_TASK2_ACCEL_MMPS2','DPT_TASK4_DIAGONAL_MMPS','DPT_TASK4_ARC_MMPS','DPT_TASK4_ACCEL_MMPS2'}
result['changed_macros']=changed
Path('/tmp/all-speed-comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result['schedule_ms'],ensure_ascii=False));print('PASS all four schedules faster; safety/EDGE macros and control algorithms unchanged.')
