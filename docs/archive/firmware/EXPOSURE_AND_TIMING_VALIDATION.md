# Exposure comparison and timing validation

Latest update: [shared exposure and report controls](DETAIL_AND_REPORT_CONTROLS.md)
supersedes the initial 14.4 dB comparison below. The current user-tuned gain
is 10000 mdB (9900 mdB applied), shared across modes, with an ASCII timing table.

## Settings for the next board run

Mode 4 now defaults to MANUAL, 5000 us exposure and 14400 mdB gain.
The IMX335 tuning header disables software AE for this policy, while AWB
remains enabled. Other capture modes retain their earlier exposure/gain defaults.
Illumination, ToF parameters, warmup, frame count, DMA routing and SD write
algorithm are unchanged.

This comparison is based on the supplied AUTO result of about 26148 us at
zero gain: exposure becomes about 5.2 times shorter and signal gain increases
by approximately the corresponding amount. It is a starting experiment, not
a guarantee of equal noise or final sharpness. The supplied 33267 us / 16800 mdB
result represents a different brightness demand and will not be matched by
the same manual settings.

With the current 4500-line frame timing, post-batch readback should show
`policy=1`, approximately `SHS1=3825`, about `675` integration lines,
`exposure_est_us` near `5000`, `gain_reg=48`, and `gain_mdB=14400`.
Small exposure differences can result from line quantization. Readback is
end-state telemetry, not exposure metadata for each of the four images.

Compare at least ten batches with the AUTO baseline, with identical lighting,
focus, working distance and image conversion. Examine stationary detail,
moving detail, noise and clipping. If motion detail improves but blur remains,
2500 us with the SAME gain isolates the shorter shutter effect but darkens the
image; approximately +6000 mdB is a separate brightness-compensation experiment
with a noise cost. Do not change several unrelated settings together.
To restore AUTO, set `CAM_EXPOSURE_MODE=0` in the Mode 4 branch; the manual
values are then inactive and ISP AE is re-enabled.

## Timing definitions

The timer starts in `Capture_RequestSnapshot`, before enqueueing the camera
request, and stops after every expected storage completion has been received.
This excludes the later ToF baseline refresh and report printing. It is not
the interval from physical insect arrival, nor the full interval until ToF
returns to its ready state. Early timeout/queue failure paths do not emit a
successful completed-cycle report.

- **Camera:** first camera-task marker through camera-function return. This
  includes frame acquisition, shutdown and deferred camera diagnostics.
- **Storage wall:** sum of all storage-command processing intervals, including
  checksum, copying, cache work, SD calls, recovery/retries and command logging.
  The per-image `[SD] OK` duration is taken before that final status print, so
  its sum can be slightly less than storage wall time.
- **Other/IPC:** cycle total minus the two measured groups. Includes dispatch,
  semaphore/queue overhead and time between storage commands. It is not silently
  called balanced when it is the largest group.
- **SD detail:** accumulated ready waits, blocking `HAL_SD_WriteBlocks` calls
  and recovery gaps. This path uses blocking writes; the report no longer
  describes them as DMA. Failed HAL calls contribute elapsed time but zero
  successful blocks. Ready waits that exit before a HAL call are included in
  storage wall and its untracked remainder.
- **Storage remaining:** storage wall minus the recorded SD detail. This is
  explicitly unclassified checksum/copy/cache/log/recovery/untracked-wait time;
  the report cannot attribute all of it to a single operation.
- **Payload:** actual successfully stored frame bytes, once per successful
  storage command. Failed images add no payload; successful retries do not
  double-count the image. HAL traffic includes successful blocks from partial
  attempts and retries, including headers/padding, so it can exceed payload.
- **Units:** binary MiB/s and KiB. 1024 blocks of 512 bytes are 512 KiB.

Phase markers preserve the first timestamp and count subsequent hits. A
repeated STORAGE or SD_WRITE marker can no longer turn a dispatch interval
into the duration up to the last frame. Only the requester finalizes DONE.
The batch error result also remains failed if an earlier image was lost but
a later image saved successfully.

`Accounting: OK` checks marker order, interval bounds and that detailed SD
counters fit within storage wall time. It is an arithmetic consistency check,
not proof of absolute hardware clock accuracy or attribution of every task
preemption. Normal serialized capture ownership is required; timeout recovery
and overlapping requests are not validated by the host arithmetic tests.

## Automated validation

Run from the project directory with host GCC on PATH:

```text
python tests/run_perf_tests.py
```

The tests compile the production reporting implementation with a fake HAL
clock, exercise a four-image cycle, retry/failed commands, tick rollover,
missing/impossible markers, repeated phases and zero-duration measurements.
They check rendered payload, throughput and batch units as well as arithmetic.
They run at debug levels 0/1/2/3, with SD subphase tracking disabled and with
running statistics disabled. Output is saved under `.validation/`.

The fixture inspired by the supplied log uses a 4839 ms cycle, 580 ms camera,
3616 ms storage wall and 1090 ms in HAL writes. Its expected results are:

```text
camera=580 ms | storage wall=3616 ms | other/IPC=643 ms
HAL calls=20 | successful blocks=19684 | max blocks/call=1024 (512 KiB)
Saved payload=9.611 MiB
HAL traffic rate=8.82 MiB/s
storage payload rate=2.66 MiB/s | cycle payload rate=1.99 MiB/s
```

These are synthetic regression expectations, not newly measured board results.
The old report had overwritten/incomplete markers, so the fixture cannot
reconstruct its complete event timeline with certainty.

ARM checks cover the changed camera, task and reporting code plus the IMX335
middleware in capture modes 0/1/2/4. The Makefile omits the ToF ULD include path
needed by app_thread.c, so validation adds the existing ULD and Discovery BSP
header paths. This does not constitute a full firmware link or a board test.

## Board acceptance checks

After rebuilding/flashing through the existing IDE workflow, collect sensor
readback, all four image IDs, per-image SD messages and the final report.
For a successful Mode 4 batch expect saved=4, failed=0, payload=9.611 MiB,
19684 successful SD blocks and normally 20 HAL calls with the current sizes.
Retries can increase HAL calls and successful traffic blocks. Verify that
camera + storage + other equals total, and that the storage remainder is
nonnegative. Compare storage wall with the four `[SD] OK` durations, allowing
for status-print time. If `Accounting: INVALID` appears, retain the complete
log and do not use its throughput or largest-group conclusion for tuning.

No firmware was flashed and no image-quality improvement is claimed as
hardware-validated by the automated tests.
