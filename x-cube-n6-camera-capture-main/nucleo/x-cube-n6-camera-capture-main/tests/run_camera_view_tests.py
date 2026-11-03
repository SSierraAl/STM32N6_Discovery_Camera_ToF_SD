"""Test the production ROI function in isolation without camera hardware."""
from pathlib import Path
import os
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
out = root / '.validation'
out.mkdir(exist_ok=True)
source = (root / 'Src/app_cam.c').read_text(encoding='utf-8')
start = source.index('static void CAM_InitCropConfig(')
end = source.index('\nstatic void CAM_EnableYuv', start)
function = source[start:end]
fixture = r'''
#include <assert.h>
#include <stdint.h>
#define MIN(a,b) ((a)<(b)?(a):(b))
typedef struct { uint32_t width,height,offset_x,offset_y; } CMW_Manual_roi_area_t;
typedef struct { int capture_width,capture_height; } CAM_conf_t;
'''
main = r'''
int main(void) {
  CMW_Manual_roi_area_t roi;
#if CAM_NATIVE_ROI
  CAM_conf_t conf = {640,768};
  CAM_InitCropConfig(&roi,2592,1944,&conf);
  assert(roi.width==640 && roi.height==768);
  assert(roi.offset_x==976 && roi.offset_y==584);
#else
  CAM_conf_t conf = {1296,972};
  CAM_InitCropConfig(&roi,2592,1944,&conf);
  assert(roi.width==2592 && roi.height==1944);
  assert(roi.offset_x==0 && roi.offset_y==0);
  assert(!(roi.offset_x & 1) && !(roi.offset_y & 1));
  assert(conf.capture_width==1296 && conf.capture_height==972);
  conf.capture_width=2592; conf.capture_height=1944;
  CAM_InitCropConfig(&roi,2592,1944,&conf);
  assert(roi.width==2592 && roi.height==1944 && !roi.offset_x && !roi.offset_y);
#endif
  return 0;
}
'''
test = out / 'camera_view_test.c'
test.write_text(fixture + function + main, encoding='utf-8')
gcc = os.environ.get('HOST_CC') or shutil.which('gcc')
if not gcc:
    raise SystemExit('Set HOST_CC to a host GCC compiler')
for mode, native in ((0, 0), (4, 0), (4, 1)):
    exe = out / f'camera_view_{mode}_{native}.exe'
    subprocess.run([gcc, '-Wall', '-Wextra', '-Werror', f'-DCAPTURE_MODE={mode}',
                    f'-DCAM_NATIVE_ROI={native}', '-DCAM_NATIVE_ROI_X=976', '-DCAM_NATIVE_ROI_Y=584',
                    str(test), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    print(f'PASS: mode={mode}, native ROI={native}, expected field and output geometry')
