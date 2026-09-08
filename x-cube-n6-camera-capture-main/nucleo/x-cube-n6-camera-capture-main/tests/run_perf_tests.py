"""Exercise timing arithmetic, table formatting and independent output switches."""
from pathlib import Path
import os
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
output = root / '.validation'
output.mkdir(exist_ok=True)
compiler = os.environ.get('HOST_CC') or shutil.which('gcc')
if not compiler:
    raise SystemExit('Host GCC not found: set HOST_CC')

# (application verbosity, SD tracking, stats window, report detail, summary enabled)
variants = [(1,1,10,2,1), (0,1,0,2,1), (3,1,10,1,1), (1,0,0,2,1),
            (0,0,0,0,1), (3,1,10,2,0), (0,0,0,1,0)]
for level, tracking, stats, detail, enabled in variants:
    tag = f'{level}_{tracking}_{stats}_{detail}_{enabled}'
    exe = output / f'perf_test_{tag}.exe'
    subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Itests/stubs', '-IInc', f'-DPERF_DEBUG_LEVEL={level}',
                    f'-DPERF_TRACK_SD_WAIT_TIME={tracking}', f'-DPERF_STATS_WINDOW={stats}',
                    f'-DPERF_REPORT_DETAIL={detail}', f'-DPERF_PRINT_SUMMARY={enabled}', '-DPERF_PRINT_STATS=1',
                    'tests/perf_timing_test.c', 'Src/perf_debug.c', '-o', str(exe)],
                   cwd=root, check=True)
    result = subprocess.run([str(exe)], cwd=root, check=True, capture_output=True, text=True)
    text = result.stdout
    assert 'PASS:' in text
    if enabled and stats: assert '[STATS]' in text
    if not enabled:
        assert '[PERF]' not in text and '|' not in text and '[STATS]' not in text
    elif detail == 0:
        assert '[PERF] #7' in text and 'total=4839 ms' in text and '|' not in text
    else:
        rows = [line for line in text.splitlines() if line.startswith('|')]
        assert rows and all(len(line) == 71 for line in rows), rows
        assert '9.611 MiB' in text and '1.99 MiB/s' in text
        assert '634 ms / 13.7%' in text and '3997 ms / 86.1%' in text
        assert '12 ms / 0.3%' in text and 'STORAGE WALL' in text
        assert 'nan' not in text.lower() and 'inf' not in text.lower()
        if detail == 2:
            assert '1024 blocks / 512 KiB' in text
            if tracking:
                assert '1090 ms' in text and '8.82 MiB/s' in text
                assert '3104 ms' in text and '10.76 MiB/s' in text
            else:
                assert 'DISABLED' in text and 'HAL traffic rate' not in text
        else:
            assert 'SD DETAIL' not in text
    (output / f'perf_test_{tag}.txt').write_text(text, encoding='utf-8')
    print(f'PASS: debug={level}, SD={tracking}, stats={stats}, detail={detail}, report={enabled}')
