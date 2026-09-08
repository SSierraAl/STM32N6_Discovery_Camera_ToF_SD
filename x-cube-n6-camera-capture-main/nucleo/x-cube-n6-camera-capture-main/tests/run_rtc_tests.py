"""Host-test production RTC code with a register-level I2C fake; test viewer metadata."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import struct
root = Path(__file__).resolve().parents[1]
out = root / '.validation'
out.mkdir(exist_ok=True)
gcc = os.environ.get('HOST_CC') or shutil.which('gcc')
if not gcc:
    raise SystemExit('Set HOST_CC to a host GCC compiler')
exe = out / 'rtc_test.exe'
subprocess.run([gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-Itests/rtc_stubs', '-IInc', '-DRTC_PRINT_TIME=0',
                'tests/rtc_test.c', 'Src/ds3231.c', 'Src/ds3231_calendar.c',
                '-o', str(exe)], cwd=root, check=True)
subprocess.run([str(exe)], check=True)
viewer_dir = next(p for p in root.parents if (p / 'raw_image_metadata.py').exists())
sys.path.insert(0, str(viewer_dir))
from raw_image_metadata import parse_header, time_label, image_filename, utc_datetime
data = struct.pack('<11I', 0x49444745,1296,972,0,2519424,1709210096,0,7,0x31435452,3,12345)+bytes(20)
h = parse_header(data)
assert '2024-02-29 12:34:56Z' in time_label(h)
assert image_filename(h,2)=='IMG_20240229_123456Z_000007_0002.png'
assert image_filename(h,2,'scaled').endswith('_scaled.png')
legacy = parse_header(data[:32]+bytes(32))
assert utc_datetime(legacy) is None and 'Legacy uptime' in time_label(legacy)
assert 'UNTIMED' in image_filename(legacy,2)
h['time_flags']=2
assert time_label(h)=='RTC unavailable'
assert parse_header(bytes(60)) is None
print('PASS: viewer UTC labels/filenames, legacy and invalid clock records')
