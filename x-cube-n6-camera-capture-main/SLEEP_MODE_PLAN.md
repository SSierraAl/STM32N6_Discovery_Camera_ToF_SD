# SLEEP-SNAPSHOT Mode (`CAPTURE_MODE = 5`) — Plan & Reference

**Board:** STM32N6570-DK (Discovery) · **Sensor:** IMX335 (CMW, CSI-2 → DCMIPP) · **RTOS:** FreeRTOS
**Goal:** put the whole board into STOP sleep and wake on the USER button
(PC13) to capture one full-quality snapshot to SD — without changing any
existing capture mode or image-quality setting.

---

## 1. Baseline (measured before this work)

| Item | Value |
|---|---|
| Idle current (legacy mode, MCU always awake, camera re-init per press) | **≈ 192 mA @ 5 V** |
| Idle power | **≈ 0.972 W** |
| Conditions | 256 MB SD card installed, ST-LINK connected |

The 192 mA baseline is dominated by the MCU + PSRAM + camera middleware
staying fully powered with all bus clocks running. STOP sleep removes the
MCU core, PLLs and bus clocks entirely.

## 2. Design overview

- **One boot, many snapshots.** SD, camera and WS2812 are initialized once
  at boot (same one-shot init that `CAPTURE_MODE = 4` already proves:
  `CAM_CallbackInit()` = full init + warmup, then sensor parked in
  **STANDBY**, DCMIPP pipe stopped).
- **MCU sleeps in STOP mode** (`HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,
  PWR_STOPENTRY_WFI)`). All clocks stop; SRAM, PSRAM, peripheral registers
  and the FreeRTOS scheduler state are **retained** — nothing is
  re-initialized on wake.
- **Wake source:** USER button PC13 = dedicated PWR wake pin **WKUP3**
  (high-level polarity, `HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH)`).
  Button is active-high with external pull-down, so a press = rising level
  = wake. The `WAKEUP_PIN_IRQHandler` clears the PWR flag.
- **Wake path:** CPU resumes on HSI → `SystemClock_Config()` re-locks
  PLL1–4 (de-static'ed in `main.c`) → sensor standby→streaming (35 ms
  settle, same proven value as mode 4) → pipe start → re-apply manual
  exposure/gain → **11 warmup frames discarded** (vsync-gated, identical
  mechanism to mode 0) → keep 1 frame → pipe stop → sensor back to
  standby.
- **Storage:** identical raw SD path as mode 0 (`SD_Reinit()` before write
  as the proven recovery, `SD_StoreRawImage()` with retry), same 64-byte
  header + YUV422 layout, same block cursor (`snap_base_block` /
  `SD_IMG_BASE_BLOCK`).
- **Image quality: unchanged.** 5 MP full resolution 2592×1944 YUV422
  (2.4 MB/frame), `SNAP_WARMUP_FRAMES = 11`, `CAM_EXPOSURE_MODE = 1`
  (manual) with `CAM_EXPOSURE_VALUE = 8 µs` + `CAM_GAIN_VALUE = 8`,
  same WS2812 capture illumination (color `0xC8B080`, 50 %).

## 3. State machine (button task, `CAPTURE_MODE = 5`)

```
                 boot: SD + camera(CAM_CallbackInit) + App_Sleep_Init
                                        |
                                        v
   +-----------------------------------+----------------------+
   |                                   |                      |
   v                                   |                      |
+SLEEP+  App_Sleep_Enter()             |                      |
|  - wait button RELEASE               |                      |
|  - WS2812 off (blocking DMA)         |                      |
|  - user LEDs off                     |                      |
|  - NOR deep power-down               |                      |
|  - console drained                   |                      |
|  - D-cache clean                     |                      |
|  - SysTick IRQ off, WFI -> STOP      |                      |
+----+--+------------------------------+                      |
       | PC13 HIGH (PWR WKUP3)                                |
       v                                                      |
+  WAKE+  SystemClock_Config() (PLL re-lock)                  |
|        NOR restored, SysTick IRQ on, D-cache invalidate     |
+----+--------------------------------------------------------+
       |
       v
+ CAPTURE+  RED LED on, WS2812_FlashStart
|          CAM_StandbySnap(): sensor->streaming (35 ms)
|          pipe start, exposure re-apply,
|          11 warmup discarded, 1 frame kept,
|          pipe stop, sensor->standby
|          WS2812_FlashStop
+----+--------------------------------------------------------+
       |                        capture failed (rc != 0)
       |                                |
       v                                v
+   SAVE+  SD_Reinit() + SD_StoreRawImage()   (wait release,
|         (mode-0 identical path, 1 retry)    settle, -> SLEEP)
+----+--------------------------------------------------------+
       |
       v
+ SETTLE+  RED off, vTaskDelay(SLEEP_ENTER_DELAY_MS = 500 ms)
+----+--------------------------------------------------------+
       |
       +-----------------------------> back to SLEEP
```

Any error path (capture fail, SD reinit fail, SD write fail after retry)
falls back to SLEEP — the device always returns to the low-power state.

## 4. Wake latency budget (estimates, no instrumentation in this pass)

| Step | Est. time |
|---|---|
| STOP exit → CPU on HSI | ~1–2 ms |
| PLL re-lock (`SystemClock_Config`) | ~1–5 ms |
| Sensor standby→streaming settle | 35 ms (`SLEEP_SENSOR_SETTLE_MS`) |
| Pipe start + 11 warmup frames @ 30 fps | ~400 ms |
| WS2812 off (blocking DMA) | ~5 ms |
| SD 2.4 MB raw write (64-block batches, 30 ms recovery gaps) | ~200–700 ms (dominant) |
| Settle before re-sleep | 500 ms |
| **Button press → back in STOP** | **≈ 1.5–2.5 s** |

FreeRTOS tick note: the SysTick IRQ is disabled around the WFI, so the
tick counter simply does not advance while asleep. No compensation is
needed (no timing-critical logic uses wall-clock time; SD timestamps in
headers are only informational).

## 5. Sleep power budget — what is on vs. off in STOP

| Subsystem | State in STOP | Est. contribution |
|---|---|---|
| STM32N6 core + PLLs + bus clocks | **off** (STOP, main regulator) | ~µA (no LSI/LPTIM1 in this design) |
| PSRAM | powered, idle (retention) | a few mA |
| IMX335 | **sensor STANDBY** (register 0x3000 = standby; CSI idle, pipe stopped) | sensor standby current (datasheet, ~tens of µA–low mA) |
| SD card + SDMMC2 | powered (rail always on), card idle, bus clock stopped | ~1–2 mA (card idle) |
| NOR flash (XSPI2) | **deep power-down** (`SLEEP_NOR_DEEP_PD = 1`) | ~µA |
| WS2812 | all LEDs off (zero frame sent, blocking) | 0 |
| User LEDs (green/red) | off (`SLEEP_LEDS_OFF = 1`) | 0 |
| UART1 console | idle (BRR retained; clock stopped) | ~0 |
| ST-LINK (if connected) | SWD pins / debugger logic | **unplugging saves several mA** |

**Expected sleep current: roughly 15–35 mA** depending on SD card,
sensor standby current and whether ST-LINK is connected — i.e. about a
**10× reduction** vs. the 192 mA baseline. The camera power rails are
always on on this board, so "camera off" here means *sensor standby*,
not hardware power-off.

### 5.1 Manual power-measurement protocol (fill in on the bench)

Setup: measure current at the 5 V input with a multimeter/probe.
Same card, same connections for every row. Record after ≥ 1 min idle
(card/sensor currents stabilize).

| # | Configuration | Current (mA) | Power @5 V (W) | Δ vs. row 1 |
|---|---|---|---|---|
| 1 | `CAPTURE_MODE=5` as built (SD in, ST-LINK in) | | | |
| 2 | Row 1 + **ST-LINK unplugged** | | | |
| 3 | Row 1 + **SD card removed** | | | |
| 4 | Row 2 + SD removed (best case) | | | |
| 5 | `SLEEP_NOR_DEEP_PD=0`, otherwise row 1 | | | |
| 6 | Reference: legacy mode (`CAPTURE_MODE=0`) idle | ~192 | ~0.972 | |

Optional ablations if more savings are wanted later:
- Sensor deep-sleep register instead of standby (needs CMW work; camera
  then needs full I2C re-init on wake → slower wake).
- STANDBY mode instead of STOP: near-zero current but full re-boot on
  wake (multi-second) — not compatible with the fast-wake goal.
- RTC/LPTIM1 periodic wake for scheduled snapshots (out of scope).

## 6. Files changed

| File | Change |
|---|---|
| `Inc/app_config.h` | `CAPTURE_MODE` → 5; mode-5 documented; new SECTION 9 (`SLEEP_*`); `PERF_DEBUG_LEVEL` quiet (0) in mode 5 only |
| `Inc/app_sleep.h` | **new** — `App_Sleep_Init()` / `App_Sleep_Enter()` API |
| `Src/app_sleep.c` | **new** — STOP entry/exit: WKUP3 (PC13, high) setup, LEDs/WS2812/NOR power-down, D-cache clean, SysTick freeze, WFI, PLL restore |
| `Inc/app_cam.h` | **new** API `CAM_StandbySnap()` |
| `Src/app_cam.c` | `CAM_StandbySnap()`: sensor wake → pipe start → re-apply exposure → 11 warmup (vsync-gated, mode-0 identical) → 1 frame → pipe stop → sensor standby. `#if CAPTURE_MODE == 5` isolated |
| `Src/main.c` | mode-5 boot branch (CAM_CallbackInit + App_Sleep_Init); `btn_sleep_fct()` sleep/wake state machine reusing the mode-0 SD save path; `SystemClock_Config()` de-static'ed (called on wake) |
| `Src/stm32n6xx_it.c` | `WAKEUP_PIN_IRQHandler()` → `HAL_PWR_WAKEUP_PIN_IRQHandler()` |
| `Makefile` (CLI) | add `Src/app_sleep.c` + sources missing for a standalone CLI link (ws2812, ToF, platform, SD HAL, FX glue) |
| STM32CubeIDE `Debug/Src/subdir.mk`, `Debug/objects.list` | add `app_sleep.c/.o` (the IDE project is folder-based, so it also picks the file up automatically on reconfigure) |

**No existing capture mode is touched:** every new code path is guarded by
`#if CAPTURE_MODE == 5` (or `#elif CAPTURE_MODE == 5`); modes 0/1/2/4
compile exactly as before.

## 7. Validation checklist

Build:
- [ ] STM32CubeIDE Debug build (or `make` in the project dir) — no new
      warnings in modified files
- [ ] Rebuild with `CAPTURE_MODE = 0` and `= 4` — compiles unchanged

Runtime (mode 5):
- [ ] Boot log shows: camera init, `Camera in STANDBY`, `Sleep module
      ready`, then `STOP mode: press USER button (PC13) to wake`
- [ ] Current drops to sleep level after boot (row 1 of §5.1)
- [ ] Press button: wake → `>>> WAKE: capturing #1...` → `[CAM] Captured
      in ~430 ms` → `>>> Snapshot #1 SAVED (block 3072, ...)`
- [ ] SD card: new ~2.4 MB raw file at block 3072 (64-byte header +
      2592×1944×2 YUV422); image converts/opens like mode-0 captures
- [ ] Repeat ≥ 5 cycles: blocks advance by `SD_BLOCKS_PER_SNAP` each time,
      no hangs, always returns to sleep
- [ ] Hold the button through a whole cycle → still captures, sleeps
      after release
- [ ] Remove SD card during sleep, press button → `SD card not ready` →
      device returns to sleep (no lockup); reinsert, next wake works
      (`SD_Reinit()` recovery path, same as mode 0)

## 8. Risks & mitigations

| Risk | Mitigation |
|---|---|
| UART byte mid-transmission when clocks stop | console is **blocking** `HAL_UART_Transmit`; drain + `SLEEP_UART_SETTLE_MS` before WFI |
| Dirty D-cache lines lost at stop | `SCB_CleanDCache()` before WFI, `SCB_InvalidateDCache()` after clock restore (STOP retains RAM/registers) |
| Tick drift while asleep | SysTick IRQ disabled across the WFI; tick simply pauses — no control logic depends on wall-clock |
| Button still held at re-sleep → immediate re-wake (level trigger) | `App_Sleep_Enter()` waits for release before WFI |
| FreeRTOS scheduler corrupted by the sleep | WFI resumes in the same task; no RTOS API is called while PRIMASK=1; all scheduler state is retained in RAM |
| Camera I2C/CSI state after clock restore | registers retained; sensor stays in standby (no I2C traffic needed until wake capture) — same proven pattern as mode 4 |
| NOR deep power-down leaves XSPI2 misbehaving | `BSP_XSPI_NOR_LeaveDeepPowerDown()` on every wake; disable with `SLEEP_NOR_DEEP_PD=0` if anything odd appears |
| WS2812 DMA in flight at WFI | `WS2812_TurnOff()` is a **blocking** DMA send; always completes before WFI |

## 9. Future work (explicitly out of scope now)

- LSI + LPTIM1 periodic wake for scheduled (unattended) snapshots
- RTC timestamp in the SD header (no RTC wake source used yet)
- Sensor deep-sleep instead of standby (more mA, slower wake)
- STANDBY mode variant for absolute minimum current (full reboot per wake)
- Power-gating the SD card / camera rails (board hardware limitation today)
