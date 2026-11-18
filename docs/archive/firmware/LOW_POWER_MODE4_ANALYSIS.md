**Mode 4 low-power design review — STM32N6570-DK Discovery**

Reviewed 2026-09-09 against the current working files. The user confirmed Discovery hardware and STM32CubeIDE as the validated build workflow. This is an engineering design and bench-validation plan; firmware was not changed, compiled, or flashed. The existing app_config.h edits were preserved. Graph discovery was used first, then current source was read directly because graph symbol ranges and mode-4 coverage were stale.

The recommended first implementation is **IMX335 sensor standby + VL53L5CX autonomous ranging + FreeRTOS-managed MCU Sleep**. Preserve boot initialization and the current mode-4 DMA sequence. Next, replace ToF polling with data-ready interrupts to retain the complete existing detector. Consider sensor threshold interrupts and MCU Stop only after measuring detection latency and energy. MCU Standby or removing camera-module power is a later, substantially more invasive option.

**What the current code actually does**

Paths below are relative to the application directory containing Inc, Src, and Documentation. Line references describe this review snapshot.

| Evidence | Current behavior | Consequence for power management |
|---|---|---|
| Inc/app_config.h | Mode 4; 1296 × 972 YUV422; four retained frames; five wake warmup frames; requested 30 fps | Preserve these settings for initial power comparisons. |
| Src/main.c:1172 | Camera initialization runs before system_ready; ToF I2C/power and RTC setup precede releasing the tasks | Keep this ordering. Routine MCU Sleep must not re-run boot initialization. |
| Src/app_cam.c:646 | CAM_CallbackInit initializes and warms the camera, stops pipe 1, then writes IMX335 register 0x3000 = 1 | Camera sensor standby already exists. This does not establish MCU sleep or removal of module power. |
| Src/app_cam.c:707 | CAM_CallbackBatchSnap writes streaming mode, delays 35 ms, starts double-buffer DMA, reapplies exposure/gain, captures, stops once, returns sensor to standby | Wrap this lifecycle with power ownership; retain the validated capture sequence. |
| Src/app_cam.c:306 | Frame-event counter is polled with a 1 ms task delay and ISP updates | Callback-driven capture still involves periodic task wakeups. Optimizing this is separate from idle power. |
| Src/app_thread.c:695 | Mode-4 camera task wakes on a 20 ms queue timeout | In mode 4 it can eventually block indefinitely for commands; modes 1/2 still service ISP while idle. |
| Src/app_thread.c:821 | Storage task already blocks on its queue | Good existing event-based behavior. |
| Src/vl53l5cx_detection.c:255 | ToF data-ready is polled over I2C every 10 ms | MCU cannot remain asleep until an insect arrives with this polling path. |
| Inc/FreeRTOSConfig.h | 1 kHz tick, idle hook disabled, no explicit tickless-idle enable; TIM4 runtime statistics enabled | Delaying a task currently does not by itself put the processor into a hardware low-power state. |
| Src/main.c:1046 | Broad EnableClockLowPower(~0) calls | These allow clocks to remain enabled in low-power operation; their names do not mean they save power automatically. |
| Src/main.c:396 | External SMPS overdrive requested; four PLLs enabled | Audit actual clock consumers and runtime frequencies before changing voltage or clocks. Comments are not a clock measurement. |
| Src/app_thread.c:867 | Illumination is stopped after camera completion, before waiting for all storage completions | This already avoids lighting throughout the entire SD-write phase. |

The current mode-4 implementation DMA-writes retained images directly into batch_buf, using two scratch buffers for warmup. Older comments about copying each frame are stale. A frame is 2,519,424 bytes; four frames are 10,077,696 bytes. Including the two scratch frames gives about 15.12 MB before the SD batch buffer and other allocations. The Discovery linker script declares a 16 MiB PSRAM region at 0x91000000; preserve that working allocation and verify the active CubeIDE linker/map before any memory power changes.

**Preserve the reliable percentage detector**

The primary sensor configuration is currently single-sensor, 4 × 4, autonomous mode, 30 ms integration, and requested 15 Hz. The active percentage threshold is 4, motion threshold 50, and minimum affected-zone count 1. Detection considers the two center rows. These are source settings, not measured operating rates.

In Src/vl53l5cx_detection.c:377, the signal test is effectively:

```c
percent_change = abs(baseline_signal - current_signal) * 100 / baseline_signal;
signal_trigger = percent_change > VL53L5CX_DET_THRESHOLD_PCT;
```

The code therefore detects both increases and decreases, despite the variable name signal_drop. Signal detection also requires a learned valid zone, accepted status 5/6/9, and current signal at least MIN_SIGNAL. Motion is an independent OR path and can trigger without those signal/status gates. Replacing this with a motion-only wake interrupt would discard the signal behavior the user finds reliable. A future signal-only policy should be an explicit detector experiment, not a hidden side effect of power optimization.

Two interrupt approaches are possible:

| Approach | What wakes the MCU | Detection equivalence |
|---|---|---|
| Data-ready interrupt — recommended first | Each completed ToF measurement, about 15/s at the requested rate | MCU evaluates every fresh result with the existing PCT, status, zone and motion logic. |
| Sensor detection-threshold interrupt — later experiment | A sensor-side candidate condition | Host must validate the candidate; equivalence must be demonstrated. |

The VL53L5CX ULD supports thresholds on signal per SPAD, including out-of-window comparisons and per-zone configurations; see [ST UM2884, section 4.12](https://www.st.com/resource/en/user_manual/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf). Local plugin files already contain SIGNAL_PER_SPAD_KCPS and OUT_OF_WINDOW constants, but the application does not configure threshold-based wakeups.

A proposed candidate window for baseline B and percentage p is approximately B × (1 − p/100) to B × (1 + p/100). For B = 1000 and p = 4, that is roughly 960–1040. It is deliberately approximate: the host uses integer division followed by a strict greater-than test, and the sensor has its own boundary and validity semantics. Start with a conservative wake filter and retain the host decision. Test exact boundary values and accepted statuses; do not claim the sensor implements the complete host algorithm.

Preserving the current OR detector also requires waking for motion candidates. At 4 × 4, the documented two-checker-per-zone facility is a candidate for signal-window OR motion; validate its motion mapping and status behavior. A filter that suppresses any event accepted by the host is unacceptable. If equivalence cannot be demonstrated, keep data-ready interrupts.

Learn the baseline at boot. Keep the monitoring ToF powered and ranging during ordinary idle; do not put it into ToF SLEEP, which stops its monitoring role. If sensor thresholds are used, refresh their absolute values after every accepted baseline refresh. Add periodic health/baseline review wakeups so the system does not become indefinitely blind after drift or an interrupt fault. Do not learn a new baseline on every insect wake: that delays capture and can learn the insect into the reference.

The physical ToF INT-to-MCU route remains unconfirmed. Inspect the actual camera-module and Discovery board revisions, connection, voltage, interrupt polarity, and pull-up arrangement. Do not infer INT wiring from LPn or I2C-reset control pins. Until INT is verified, software Sleep can still be tested with the existing timed polling.

**Choose MCU power depth in stages**

STM32N6 Sleep stops the CPU clock while configured peripherals can run. Stop needs clock restoration; Stop SVOS HIGH supports EXTI wake, whereas the SVOS LOW wake-source set is more restricted. BSEC clock enable is a Stop-entry requirement. Standby has a reset-style restart and retention constraints. These distinctions come from [ST AN5946, sections 4.1 and 5.3](https://www.st.com/resource/en/application_note/an5946-how-to-optimize-lowpower-modes-on-stm32n6-mcus-stmicroelectronics.pdf).

| Stage | Proposed idle state | Why / acceptance criterion |
|---|---|---|
| 1 | MCU shallow Sleep; camera standby; ToF unchanged | Lowest integration risk; initialization and memory mapping survive. Demonstrate lower measured idle energy with unchanged captures. |
| 2 | Tickless Sleep + ToF data-ready interrupt | Remove unnecessary polling wakeups without changing the detector. Verify no lost measurements or timing regressions. |
| 3 | Reduced idle clocks and selective clock gating | Reduce remaining clock-tree consumption. Restore full capture performance before camera wake. |
| 4 | Stop with retained application state | Only after board-specific wake, memory, regulator and RTOS-time restoration are proven. |
| 5 | MCU Standby or camera power removal | Consider for long predictable inactivity or an upstream trigger with adequate lead time. Requires a restart design and measured energy benefit. |

Use the scheduler's idle integration, not a HAL sleep call inside sensor_task while other tasks may have work. The selected local Cortex-M55 FreeRTOS port contains vPortSuppressTicksAndSleep and race checks. Its standard implementation uses SysTick with a 24-bit interval limit. It is a starting point for shallow Sleep, not a ready-made Stop implementation. [FreeRTOS low-power documentation](https://www.freertos.org/low-power-ARM-cortex-rtos) describes enabling supported ports with configUSE_TICKLESS_IDLE.

For Stage 1, a nonblocking idle-hook WFI with SLEEPDEEP cleared is a small initial option, keeping normal ticks and clocks. Then enable and verify the port's tickless shallow-Sleep path. Do not clear real pending sensor events during entry. Ensure selected peripheral sleep clocks preserve XSPI memory access, ToF I2C, and any active capture/storage DMA. Shallow Sleep during DMA may be valid; deep Stop during capture or storage is prohibited by the proposed application policy.

For Stop, use a verified always-running low-power time source and account elapsed time back into FreeRTOS. HAL_GetTick currently returns FreeRTOS ticks and HAL_Delay calls vTaskDelay. Calling existing SystemClock_Config from an interrupt-masked wake hook is unsuitable: it invokes HAL_Delay and boot-time regulator setup. Implement a dedicated bounded restore path that does not require the suspended scheduler or a stopped tick for timeouts. Restore clocks and memory access before releasing interrupts/tasks that depend on them. The DS3231 supplies calendar timestamps; this code does not configure it as a wake source or provide subsecond RTOS sleep accounting.

The Discovery board defaults to an external SMPS; current firmware selects overdrive through BSP_SMPS_Init. Board revision and supply configuration must govern Stop/voltage settings, rather than copying internal-SMPS examples. [ST UM3300, section 7.4.5](https://www.st.com/resource/en/user_manual/um3300-discovery-kit-with-stm32n657x0-mcu-stmicroelectronics.pdf) documents the alternatives. Audit all consumers before disabling a PLL, including CSI/DCMIPP, XSPI, SDMMC, UART and timers.

**State ownership and sleep boundary**

Proposed application sequence:

```text
BOOT_INIT -> ARMED
ARMED -- ToF result/candidate --> VALIDATE
VALIDATE -- no detection --> ARMED
VALIDATE -- accepted detection --> WAKE_CAMERA -> CAPTURE_BATCH
CAPTURE_BATCH -> CAMERA_STANDBY + LIGHT_OFF -> SAVE_BATCH
SAVE_BATCH -> CARD_READY -> REARM_WINDOW -> ARMED
Any failure -> RECOVER -> ARMED only after hardware is known idle
```

ARMED permits MCU sleep while the ToF ranges. REARM_WINDOW is a timed monitoring state; the MCU can sleep between measurements during those few seconds. A delay is not a reason to keep camera streaming, LEDs lit, or the CPU spinning. If measurements show that repeated nearby insects justify keeping the camera streaming briefly, add that as a separate policy with a measured energy/latency tradeoff.

Introduce an app_power module for policy and board restore operations. Keep sensor_task as ToF owner, camera_task as camera owner, and storage_task as storage owner. Use synchronized busy references or state/event bits covering camera activity, storage, I2C operations, baseline learning and recovery. A volatile g_capture_busy flag alone does not prove the peripherals are safe for Stop.

Before deep sleep, establish all of the following atomically with the sleep-entry handshake: no accepted capture pending; no frame DMA active; camera standby write succeeded; all storage commands and in-flight writes completed; SD reports ready; shared I2C is idle; illumination DMA has stopped; required wake sources are armed; no pending ToF result is being discarded. The wake ISR should latch/notify only, with a FreeRTOS-compatible interrupt priority. Camera initialization, I2C transactions and SD writes remain in task context.

Keep PSRAM powered and accessible during initial stages. Only later consider its device-specific low-power modes once the active linker map proves that all live stacks, code and required state remain available. Clean/invalidate caches according to ownership and memory-state transitions. Module-wide power removal also needs rail/schematic verification because the camera assembly contains the ToF; disabling its supply may disable the detector.

**Existing failure paths to strengthen before Stop**

1. CAM_CallbackInit and CAM_CallbackBatchSnap currently ignore several sensor-register and pipe-stop return codes. CAM_IsCallbackReady reflects successful initialization, not proof of physical standby. Track initialized/streaming/standby/fault states explicitly and require successful quiescence.
2. Capture_RequestSnapshot waits for BATCH_FRAMES storage completions when no frames were captured. That can wait until timeout for writes never queued. Return an explicit capture result and the actual queued-frame count, including queue-send failures.
3. A pipeline timeout does not prove the camera/storage task stopped. Add cancellation/recovery acknowledgement before releasing buffers or permitting Stop; avoid stale completion tokens crossing capture generations.
4. SD_StoreRawImage polls readiness before writes, but returns after the last write without an explicit final SD_WaitForReady. Add a bounded final card-ready check before deep sleep or SD clock/power changes. Continue to honor error/retry handling.
5. Single-sensor cooldown is five successful update frames, not a fixed number of milliseconds. Changing ToF rate changes its duration. Use a wrap-safe time deadline for a requested seconds-based cooldown, with explicit rearm/scene-clear behavior. Continue monitoring during cooldown.
6. The current adaptive baseline refresh follows repeated captures. Test it with a target remaining present, since relearning an occupied scene can change future detection behavior. Power-state transitions should not silently trigger baseline relearning.

**Latency can be more limiting than MCU wakeup**

At the configured 30 fps, five discarded frames plus the first retained frame take about 200 ms. Adding the existing 35 ms wake delay gives approximately **235 ms to first retained frame completion**, before extra sensor synchronization, software overhead or MCU clock restoration. The full five-plus-four frame sequence is approximately **335 ms**, with the same qualifications. These are estimates from code settings, not measurements. ToF at 15 Hz adds a frame period of roughly 66.7 ms as a sampling scale; integration and detection behavior require measurement.

Therefore, moving from MCU Sleep to a deeper mode may be a small part of the total photographic delay. Measure time from physical insect entry and ToF indication to the first useful exposure, not only to the callback. If an insect crosses the useful field of view faster than this budget, upstream detection or keeping the camera streaming during an approach window may be necessary. Lead distance is approximately insect speed × required lead time, with margin. Do not reduce warmup frames or ToF rate until missed-insect rate and first-frame quality have been compared.

Manual exposure is currently 15000 microseconds (15 ms), despite an adjacent 10 ms comment. Sleep optimization does not correct motion blur. Preserve exposure, gain and lighting for the power comparison; optimize image quality as a separate experiment.

Dual-sensor mode is available but currently disabled. Its guardian can provide advance notice only if placement provides enough lead time. Present primary wake/baseline-learning delays need measurement; simply enabling dual mode does not guarantee timely photographs.

**Bench plan and implementation order**

| Experiment | Keep fixed | Record / pass condition |
|---|---|---|
| Current mode-4 reference | Detector, exposure, gain, light, SD card, target movement | Idle power, complete-cycle energy, trigger-to-first-good-frame, accepted/missed detections, SD errors |
| Shallow Sleep only | All application behavior and clocks | Lower idle energy; repeated captures and button behavior unchanged |
| Tickless + queue/button cleanup | Same ToF rate and algorithm | Fewer wakeups; monotonic timing; no lost commands |
| ToF data-ready IRQ | Same detector | Same results on recorded/scoped events; interrupt remains reliable after capture and baseline refresh |
| Sensor threshold candidate filter | Host detector retained | No missed host-positive events across motion, signal increase/decrease, invalid-status and boundary cases |
| Stop and restore | Validated capture profile | Repeated wake cycles, PSRAM integrity, I2C recovery, timestamps, card-ready completion and measured energy benefit |

Run at least hundreds of capture/sleep cycles before a long unattended soak. Include target present at wake/rearm, immediate consecutive insects, no insect for hours, failed camera start, absent/slow SD card, missed/stuck INT, and a trigger arriving exactly during sleep entry. GPIO timing markers and a current trace are more useful for wake timing than UART print timestamps.

Measure at the actual system supply for battery budgeting, and separately measure MCU rails if diagnosing savings. CN2 provides selected MCU-rail measurement points, with resistor configuration requirements in [UM3300, sections 7.4.6 and 8.2](https://www.st.com/resource/en/user_manual/um3300-discovery-kit-with-stm32n657x0-mcu-stmicroelectronics.pdf). A VDDCORE measurement excludes other rails and board peripherals. Compare debugger-attached and deployed operation; account for ST-LINK, display/backlight, Ethernet, regulators, SD standby and the camera-module supply.

For a representative interval T, use measured complete event energy E_event, event count n, event active time t_event, and idle power P_idle:

```text
E_total = P_idle * (T - n * t_event) + n * E_event
P_average = E_total / T
```

Include false triggers in n. For a deeper sleep transition with additional energy E_transition, its approximate break-even idle duration is E_transition / (P_shallow − P_deep), provided the denominator is positive. A lower MCU current is not sufficient if wakeup loses insects or increases whole-system energy.

For a board-specific implementation reference, ST's [x-cube-n6-ai-power-measurement](https://github.com/STMicroelectronics/x-cube-n6-ai-power-measurement) demonstrates Discovery-board CPU Sleep during camera capture, clock scaling and unused-peripheral deinitialization. Its camera lifecycle differs from this project's preserved mode-4 initialization; reuse verified power techniques selectively.

The next firmware patch should be a small, reversible mode-4 shallow-Sleep experiment, built by the user in STM32CubeIDE, followed by energy and image comparison. ToF INT routing, board/module revision, physical insect dwell time, and measured baseline power remain the key inputs before choosing the final Stop implementation.
