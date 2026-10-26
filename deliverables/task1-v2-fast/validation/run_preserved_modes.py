from pathlib import Path
import subprocess,sys
root=Path.cwd();tests=root/'tools/tests'
sources=['/tmp/sunward_preserved_modes.c']+[str(p) for p in sorted((root/'Core/Src').glob('car_*.c'))]+[str(root/'Core/Src/dual_pi_test.c')]
subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-DCAR_APP_TEST','-I'+str(tests),'-I'+str(root/'Core/Inc')]+sources+['-lm','-o','/tmp/sunward_preserved_modes'],check=True)
p=subprocess.run(['/tmp/sunward_preserved_modes'],capture_output=True,text=True);Path(sys.argv[1]).write_text(p.stdout+p.stderr);p.check_returncode()
print(p.stdout)
