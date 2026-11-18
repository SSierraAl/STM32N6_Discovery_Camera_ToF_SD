# Camera, RTC and report controls

Edit `Inc/app_config.h`, rebuild and flash using the existing CubeIDE project.
The default remains **Mode 4**, four 1296x972 frames, exposure **5000 us**,
gain **12000 mdB**. No optional centre crop remains.

## One full-resolution photograph

Change only:

```c
#define CAPTURE_MODE 0
```

Leave `CAM_BINNING` on its automatic mode-dependent default. Mode 0 selects
2592x1944; Mode 4 selects 1296x972. Remove any separate compiler override of
CAM_BINNING when switching modes. After boot, press USER (PC13) once to save
one full-field 2592x1944 YUV422 photo. Return CAPTURE_MODE to 4 for normal
ToF-triggered callback batches. Mode 0 is a button test, not a ToF single-shot mode.

The legacy name CAM_BINNING selects DCMIPP output scaling; it does not enable
physical IMX335 binning. Both settings retain the full field of view.

| Configuration | Camera buffers plus SD staging | Saved payload per trigger |
| --- | ---: | ---: |
| Mode 0: one full-resolution photo | 10,601,984 bytes (10.11 MiB) | 10,077,696 bytes |
| Mode 4: four half-resolution photos | 15,640,832 bytes (14.92 MiB) | 10,077,696 bytes |

Mode 0 reuses one buffer for all warmups. Mode 4 uses two scratch buffers and
four final slots. The IDE linker `Gcc/STM32N657xx.ld` provides 16 MiB of PSRAM;
these figures cover the camera/SD arrays, not every possible linker allocation.
Full-size Mode 4 remains rejected: six full-size buffers would exceed 16 MiB.

The single path now waits for frame-complete events instead of VSYNC, checks
start/stop results, and rejects reported Pipe 1 errors. The Mode 4 acquisition
loop is unchanged. Full-size output increases instantaneous DMA traffic: a
board run is still required to validate bandwidth, clean frame edges and fine
detail. Equal total SD bytes does not guarantee equal storage or capture time.
Keep the same 5 ms exposure, gain and lighting for the first comparison.

## Show or hide the performance table

```c
#define PERF_PRINT_SUMMARY 1  /* 0 = hide the report */
#define PERF_REPORT_DETAIL 2 /* 0 = one line, 1 = basic table, 2 = full table */
```

These apply to the task-based capture report (including Mode 4). The legacy
Mode 0 button path prints its separate capture/SD/total timing line.

Other switches are independent:

- PERF_DEBUG_LEVEL: general application verbosity.
- PERF_CAMERA_FRAME_LOG=0: suppress per-frame waits.
- CAM_SENSOR_REG_DEBUG=0: suppress post-batch sensor register reads.
- PERF_PRINT_STATS=0: suppress rolling statistics.
- PERF_TRACK_SD_WAIT_TIME=0: suppress detailed SD counters, retain storage wall time.
- RTC_PRINT_TIME=0: suppress per-capture clock logs; still save timestamps.

RTC reads/logs occur after the camera-end marker and before storage dispatch.
Their small cost therefore appears in other/IPC, not camera acquisition or
blocking SD writes. No RTC transaction is added to the Mode 4 frame callback.

## DS3231 clock on the external ToF bus

The driver uses `hi2c1` and the existing `g_i2c1_mutex`. HAL uses address 0xD0
(the 7-bit address 0x68 shifted left once). The AT24C32 at 0x57 is unused.
Clock initialization runs after I2C/ToF power-up and before capture tasks start.
The RTC keeps time independently while ToF ranging is stopped. It needs a
working backup supply to retain time when the module's main power is removed.

The driver reads the calendar and status in one coherent 16-byte transaction.
An oscillator-stop flag, disabled battery oscillator, invalid calendar or I2C
failure makes the timestamp unavailable. Reading an unset clock does not clear
its validity flag or pretend uptime is UTC. Reads and mutex waits are bounded.
The software supports 2000 through 2099 and stores UTC in 24-hour format.

### If the log says UNTIMED

Read the startup diagnostic first. I2C transfer failure (including HAL error
0x00000004, NACK) is different from a responding clock with its oscillator-stop
flag set. Only the latter is fixed by provisioning the time. A mutex-unavailable
message means the read could not obtain the shared bus within its timeout.

The supplied board log lists 0x57 and 0x69, but not 0x68. A DS3231 has fixed
7-bit address 0x68 (HAL 0xD0); do not substitute 0x69 based only on this scan.
Check the chip marking, module power/common ground, and SDA/SCL connections to
the shared bus (the firmware reports PC1 SDA / PH9 SCL). The WS2812 OFF log is
from the illumination driver, not an RTC acknowledgement. An illuminated module
LED also cannot establish successful communication with the RTC chip.

After reconnecting/rebooting, check that the scan includes 0x68 and review the
new RTC HAL/error diagnostic. Once the device responds, provision UTC below if
it reports oscillator-stop/invalid calendar. The firmware does not change the
camera pipeline or probe/write an unknown device at 0x69 to work around this.

### Set your currently unset clock

1. Set RTC_SET_UTC to the intended UTC date/time, for example
   `"2026-09-08T14:30:00Z"` (replace this example with your actual UTC time).
2. Set RTC_SET_ON_BOOT to 1, rebuild and flash. On boot, look for
   `[RTC] Time set explicitly` followed by `[RTC] DS3231 UTC ...`.
3. Set RTC_SET_ON_BOOT back to 0, rebuild and flash again. Later boots preserve
   the battery-backed clock. Leaving the flag at 1 resets time on every boot.
4. Verify printed UTC against a known clock, then power-cycle and verify it
   advances. The compile/flash delay limits this provisioning method's accuracy;
   `RTC_Set()` is also available for a future interactive synchronization command.

Default configuration does **not** write a date. RTC_SET_UTC deliberately starts
with an invalid placeholder. With the unset clock, images are saved as UNTIMED
until you provision it. RTC_ENABLE=0 disables application RTC access entirely.

Reference: [Analog Devices DS3231 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ds3231.pdf),
calendar buffering, control EOSC and status OSF descriptions.

## Photo timestamps and direct raw SD storage

No FileX, filesystem formatting, EEPROM writes or pixel overlays were added.
The existing raw layout stays at 64 header bytes followed by the YUV payload.
The old header structure was only 60 bytes despite a 64-byte copy; it is now
exactly 64 bytes, enforced by a compile-time assertion.

The first eight uint32 fields retain their offsets. New fields use the reserved
header space (little endian):

| Byte offset | Meaning |
| --- | --- |
| 20 | Unix UTC seconds when valid; zero when unavailable |
| 32 | RTC1 marker, 0x31435452 |
| 36 | Flags: bit 0 UTC valid; bit 1 timestamp is post-capture completion |
| 40 | MCU uptime in ms sampled immediately before the clock read |
| 44..63 | Reserved, zero |

One RTC read occurs **after each successful capture/batch**, before queueing
storage. All four images in a batch share that second-resolution completion
time and carry different image IDs. This is not a per-frame exposure timestamp.
Retries reuse the same metadata. Camera-side ID reservation avoids a race with
the storage task, and failed writes may leave ID gaps.

Run the updated root `SD_Image_Viewer.py` with `raw_image_metadata.py` beside it.
New exports use names such as `IMG_20260908_143000Z_000007_0002.png` (image ID,
then scan index). The index also distinguishes old firmware records with ID 0.
The viewer displays old timestamps as legacy uptime, not a date in 1970.
Invalid-clock records display RTC unavailable and export with UNTIMED names.

The existing SD allocation policy is retained: boot restarts at
SD_SNAP_BASE_BLOCK, so this change does not add append-after-reboot retention.
Export captures before rebooting if you need to retain them. The timestamp
header is metadata, not a filesystem filename or an append journal.

## Why the SD log changed from about 800 ms to 1050 ms

The newer per-image timer begins before the full-image checksum. The older
marker was after it, so those numbers do not measure the same scope. Retries
also contribute to wall time. This does not establish how much of your measured
difference comes from accounting versus card/build/cache variation.

The full table reports `Of remaining: checksum` within remaining storage work;
it is a subtotal, not an extra group. Compare storage wall, checksum and HAL
write times on repeated captures. Table printing happens after storage and is
excluded from the measured storage duration. The SD write algorithm, staging
size and recovery gap have not been tuned in this change.

## Validation

Host tests: run `python tests/run_perf_tests.py`,
`python tests/run_camera_view_tests.py` and `python tests/run_rtc_tests.py`.
Set HOST_CC to a host GCC executable if necessary. Tests cover table controls,
timing arithmetic, UTC/BCD conversion, leap dates, OSF and transport failures,
shared-bus locking, metadata layout and viewer naming/legacy compatibility.
ARM syntax checks cover the modified C files in modes 0, 1, 2 and 4.
These are software checks; firmware has not been flashed and full-resolution
image quality, DMA bandwidth and RTC battery retention remain board checks.
