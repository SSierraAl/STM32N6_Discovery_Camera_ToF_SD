/**
 ******************************************************************************
 * @file    app_config.h
 * @author  GPM Application Team (cleaned for standalone snapshot mode)
 *
 * @brief   Central configuration for standalone camera snapshot capture.
 *
 *   This project captures images from the CMW-IMX335 camera on the
 *   NUCLEO-N657X0-Q board and stores them as raw YUV422 frames on an
 *   SD card via raw block I/O (no FileX/fatfs filesystem needed).
 *
 *   Flow:
 *     Boot → Init SD card → Wait for USER button (PC13)
 *       → Power on camera → Discard N warmup frames → Capture frame #N+1
 *       → Save to SD → Power off camera → Wait for next press
 ******************************************************************************
 */
#ifndef APP_CONFIG
#define APP_CONFIG

/* ================================================================
   SECTION 1: CAMERA CAPTURE PARAMETERS
   Change these to adjust snapshot behavior.
   ================================================================ */

#ifndef CAPTURE_MODE
#define CAPTURE_MODE            5
#endif

/** Capture mode for ToF-triggered photography.
     0 = ON-DEMAND  — Legacy button/manual single-frame mode. //Validated !
    1 = CONTINUOUS — Camera always running in continuous mode at low resolution.
                 On trigger: grab one completed frame while pipe stays running.
                 Used with ToF enabled.
     2 = BATCH      — Camera always running. Captures BATCH_FRAMES per detection into PSRAM,
                     then writes to SD sequentially. FASTEST capture, max data collection.
                     NOTE: Has I2C conflict with ToF — CMW_CAMERA_Init fails (ret=-7).
     4 = CALLBACK-BATCH — Camera init+warmup ONCE at boot, stays in standby between triggers.
                     On ToF trigger: wake → start continuous pipe → warmup using FRAME EVENT CALLBACK
                     (g_frame_event_count incremented by DCMIPP ISR) → capture N frames WITHOUT
                     stopping the pipe between them → memcpy each frame as callback fires →
                     stop pipe once at end → return to standby.
                     ALL frames are sharp because the pipe never restarts mid-batch.
                     Uses the SAME callback infrastructure as STANDBY-BATCH but eliminates
                     the Stop/Restart tearing that causes blurry frames.
     5 = SLEEP-SNAPSHOT — Camera init+warmup ONCE at boot, then the MCU enters
                     STOP sleep. USER button (PC13 = PWR WKUP3, HIGH level) wakes
                     the CPU; PLL clocks are restored, the sensor wakes from standby,
                     ONE full-res frame is captured (same warmup/exposure as mode 0),
                     saved to SD with the same raw layout, and the MCU returns to STOP.
                     Lowest idle power — see SLEEP_MODE_PLAN.md.
     Snapshot mode is the same as ON-DEMAND (CAPTURE_MODE = 0). */


/** Camera binning mode — simple toggle to switch resolutions.
     0 = FULL RESOLUTION (2592x1944, 5MPX) — Use this for snapshot mode
     1 = 2x2 BINNING (1296x972, 1.3MPX) — Faster readout, less rolling shutter */
#if CAPTURE_MODE == 1 || CAPTURE_MODE == 2 || CAPTURE_MODE == 4
#define CAM_BINNING          1
#else
#define CAM_BINNING          0
#endif

/** Snapshot resolution (auto-calculated from CAM_BINNING).
    YUV422 format = 2 bytes per pixel */
#if CAM_BINNING == 1
    #define SNAP_WIDTH       1296   /* 2x2 binned: 2592/2 */
    #define SNAP_HEIGHT      972    /* 2x2 binned: 1944/2 */
#else
    #define SNAP_WIDTH       2592   /* Full resolution */
    #define SNAP_HEIGHT      1944   /* Full resolution */
#endif

/** Camera frame rate (FPS).
    Valid values depend on resolution:
      - 640x480  : 30 or 60 FPS
      - 1024x768 : 30 FPS only
      - 1920x1080: 15 or 30 FPS
      - 2592 x 1944 max */
#define SNAP_FPS             30

/** Number of warmup frames to discard after camera power-on.
     The IMX335 sensor outputs garbage on the first frames.
     REDUCED FROM 11 → 8: Saves ~133ms per capture (33ms saved per frame).
     CRITICAL: Each warmup frame = ~33ms of delay where the insect can move!
     If images have green tint on first few captures, increase this value. */
#define SNAP_WARMUP_FRAMES    11

/** Maximum time to wait for warmup + capture frames (milliseconds). */
#define SNAP_TIMEOUT_MS      200

/* ================================================================
   SECTION 1B: CAMERA IMAGE QUALITY (exposure, gain, brightness)
   Tune these to match your lighting conditions.
   ================================================================ */

/** Exposure mode:
    0 = AUTO (sensor decides, can be slow in low light = motion blur)
    1 = MANUAL (fixed exposure, faster = less motion blur)
    2 = FREEZE  (use last auto value, good for consistent lighting)

    IMPORTANT: For burst capture across standby wake (CAPTURE_MODE 4),
    MANUAL (1) is strongly recommended over AUTO/FREEZE. In AUTO or
    FREEZE the software 3A (AE/AWB) library keeps re-converging (or freezes
    onto whatever value it happened to be at, which is not deterministic
    across standby/wake cycles), which is what was causing shot-to-shot
    brightness/color drift and inconsistent sharpness. MANUAL pins exposure
    and gain to fixed values every single capture, giving repeatable
    brightness/color and a guaranteed-fast shutter to freeze motion. */
#define CAM_EXPOSURE_MODE    1

/** Exposure time in MICROSECONDS (only used in MANUAL mode).
      The IMX335 sensor expects exposure in µs.
      Valid range: 8-33266 µs (IMX335_EXPOSURE_MIN to IMX335_EXPOSURE_MAX).

      NOTE: At 30 FPS, one frame period = 33,333 µs. Exposure cannot exceed this.
      At 8 µs: Shutter = VMAX - 1 line = FASTEST POSSIBLE (freezes vibration blur).

      IMPORTANT: Check console output for "[CAM] Readback: exposure=XXX"
      If readback differs from configured value, sensor driver is clamping.

      CRITICAL BUG FIX: The ISP library overwrites exposure during startup.
      app_cam.c now re-applies exposure AFTER CAM_CapturePipe_Start().
      Check console for "[CAM] Post-start exposure=XXX" to verify.

      For FAST SHUTTER (freeze vibrating objects): use minimum exposure 8 µs
      with moderate GAIN (2000-4000). Good lighting required.

      TUNING NOTE (2026-07-16): confirmed via readback that 8 µs was genuinely
      applied, yet images were still soft/blurry. At 8 µs the sensor collects
      ~25x less light than at 200 µs, and 200 µs is still >150x faster than a
      typical 1/1000s "frozen motion" shutter speed - more than fast enough
      for an insect, which moves a negligible sub-pixel distance in 200 µs.
      Going shorter than necessary only starves the sensor of light, forcing
      more analog gain and more visible noise, which looks like blur/softness.
      Raised to 200 µs paired with brighter illumination (see WS2812_ILLUMINATION_BRIGHTNESS)
      and reduced gain below, for a much better signal-to-noise ratio while still
      easily freezing motion. Tune down toward 8 if you confirm actual motion blur
      (not noise) at 200 µs; tune up if still too dark. */
#define CAM_EXPOSURE_VALUE   8

/** Analog gain (only used in MANUAL mode).
      Range: 0-72000 (IMX335_GAIN_MIN to IMX335_GAIN_MAX).
      Gain is internally represented as value * 1000 (e.g., 2000 = 2.0x gain).
      Higher = brighter image but more noise/grain.

      With 8µs exposure (fastest shutter), the sensor collects very little light,
      so gain MUST be high enough to produce a usable image.
      Good starting points:
        - 8µs exposure + bright light:    2000-4000 (2x-4x gain)
        - 8µs exposure + medium light:    4000-8000 (4x-8x gain)
        - 1000µs exposure:                200-600

      TUNING NOTE (2026-07-16): lowered alongside the CAM_EXPOSURE_VALUE and
      WS2812_ILLUMINATION_BRIGHTNESS increase - with ~25x more exposure time
      and ~8x more LED brightness, far less analog gain is needed to reach the
      same brightness, and less gain means less sensor noise (sharper-looking
      images). Raise back toward 4000 if images come out too dark. */
#define CAM_GAIN_VALUE       8

/** Brightness adjustment.
    Range: depends on sensor (typically -128 to +127).
    0 = default. Positive = brighter, negative = darker.
    Default: 0 */
#define CAM_BRIGHTNESS       0

/** Contrast adjustment.
    0 = default. Use small positive values only if the image looks flat.
    Default: 0 */
#define CAM_CONTRAST         0

/** Anti-flicker mode (for AC-powered lighting).
    0 = disabled, 1 = 50Hz (EU), 2 = 60Hz (US/Japan), 3 = auto
    If images have horizontal banding under fluorescent lights,
    set this to match your mains frequency.
    Default: 0 (disabled) */
#define CAM_ANTI_FLICKER     0

/* ================================================================
   SECTION 2: MEMORY / BUFFER SIZES (auto-calculated)
   Do NOT change unless you understand PSRAM layout.
   ================================================================ */

/** Frame buffer size in bytes = width * height * 2 (YUV422) */
#define SNAP_FRAME_SIZE      (SNAP_WIDTH * SNAP_HEIGHT * 2)

/** Maximum PSRAM buffer for snapshot capture (same as frame size) */
#define MAX_SNAP_FRAME_SIZE  SNAP_FRAME_SIZE

/* ================================================================
   SECTION 3: SD CARD STORAGE PARAMETERS
   ================================================================ */

/** SD card block size (standard) */
#define SD_BLOCK_SIZE        512

/** Header size prepended to each frame on SD card (bytes) */
#define SD_IMG_HEADER_SIZE   64

/** Total blocks per snapshot = ceil((header + frame) / 512) */
#define SD_BLOCKS_PER_SNAP   ((SD_IMG_HEADER_SIZE + SNAP_FRAME_SIZE + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE)

/** First SD block used for snapshots (leaves blocks 0..3071 free) */
#define SD_SNAP_BASE_BLOCK   3072

/** Number of SD blocks written per HAL_SD_WriteBlocks call.
    For 5MP (10MB) = 19684 blocks total:
    - 32 blocks/call = 615 HAL calls (safer for slow cards)
    - 64 blocks/call = 307 HAL calls
    - 128 blocks/call = 154 calls (too aggressive, causes CRC errors)

    SD card needs time between batches for internal flash programming.
    Use smaller batches + longer waits = more reliable. */
#define SD_BATCH_WRITE_BLOCKS 64 //128 in but mode - 64 in batch mode is  more safe

/** Minimum inter-batch recovery gap (milliseconds).
    After the SD card reports TRANSFER-ready (via CMD13 poll), we wait this
    many additional milliseconds before sending the next write command. The
    card's CMD13 "ready" status is optimistic — the host controller reports
    ready before the internal NAND flash erase/program cycles complete.
    Without this gap, STA=0x5000 (Data CRC timeout) errors occur on many
    SDXC cards when the next write arrives too early.

    Recommended values:
    - 15ms: good balance of speed and reliability (default)
    - 20ms: more reliable for slower/failing cards
    -  0ms: fastest but may cause CRC errors on some cards
    - 30ms+: only if you still see CRC failures at 20ms */
#define SD_BATCH_RECOVERY_GAP_MS  30 //5 but version - 30 batch mode  is more safe together with 64

/** Maximum snapshots that can be stored before SD card overflow.
    For a 32 GB SDHC card (64,000,000 blocks):
    (64,000,000 - 3072) / 19,643 ≈ 3,257 snapshots max.
    This is a safety constant; the actual limit depends on card size. */
#define SD_MAX_SNAPSHOTS       3000

/* ================================================================
   SECTION 4: CAPTURE MODE SELECTION
   ================================================================ */



/* ================================================================
    SECTION 5D: CALLBACK-BATCH PARAMETERS (CAPTURE_MODE = 4)
    ================================================================ */

/** Callback-based batch capture: uses DCMIPP frame event callback (ISR-driven)
     to know exactly when each frame is ready. The pipe runs continuously through
     the entire batch — no Stop/Restart between frames = no tearing = ALL frames sharp.
     
     Flow on trigger:
       Wake sensor (20ms) → Start pipe → Wait for warmup frames via callback →
       For each capture frame: wait for callback → memcpy → repeat →
       Stop pipe once → Return to standby
     
     Timing: warmup(N) + capture(M) frames at ~33ms each = (N+M)*33ms total.
     With CALLBACK_WARMUP=5 and CALLBACK_FRAMES=3: ~264ms from wake to SD write. */
#define CALLBACK_WARMUP_FRAMES   11     /* Frames to discard after wake (callback-based) */
#define CALLBACK_FRAMES          4   /* Number of frames to capture per trigger */
#define CALLBACK_WAKE_TIMEOUT_MS 1000  /* Max wait for wake + warmup + capture */

/** Total PSRAM needed for callback batch: CALLBACK_FRAMES × frame_size
     At 1296x972 YUV422: 3 × 2.5MB = 7.5MB (fits in 16MB PSRAM) */
#define CALLBACK_BUF_SIZE        (CALLBACK_FRAMES * SNAP_FRAME_SIZE)

/* ================================================================
    SECTION 5B: BATCH CAPTURE PARAMETERS (CAPTURE_MODE = 2)
    ================================================================ */

/** Number of frames captured per insect detection event.
     Each frame is stored in PSRAM (batch_buf), then written to SD sequentially.
     More frames = higher chance of at least one sharp image, but more PSRAM used
     and longer total SD write time (happens AFTER capture, so insect can leave).

     PSRAM usage: BATCH_FRAMES × SNAP_FRAME_SIZE
       - 1296x972@YUV422 (2.4MB/frame): 3 frames = 7.2MB, 5 frames = 12MB
       - 2592x1944@YUV422 (9.6MB/frame): 3 frames = 28.8MB, 5 frames = 48MB

     Capture time: ~38ms per frame (stop pipe + memcpy + restart + wait 1 frame)
       - 3 frames = ~114ms total capture window
       - 5 frames = ~190ms total capture window

     Recommended: 3 (good balance), 5 (max data, still fast) */
#define BATCH_FRAMES             3

/** For CALLBACK-BATCH mode (CAPTURE_MODE=4): override BATCH_FRAMES to use CALLBACK_FRAMES.
     This ensures app_thread.c uses the correct frame count for SD storage. */
#if CAPTURE_MODE == 4
#undef BATCH_FRAMES
#define BATCH_FRAMES             CALLBACK_FRAMES
#endif

/** Total batch buffer size in bytes = BATCH_FRAMES × SNAP_FRAME_SIZE.
    This buffer is allocated in PSRAM and holds all frames for one detection event. */
#define BATCH_BUF_SIZE           (BATCH_FRAMES * SNAP_FRAME_SIZE)

/* ================================================================
   SECTION 5: BUTTON / LED PARAMETERS
   ================================================================ */

/** Debounce time for USER button press detection (milliseconds).
    Reduced from 50 to 20 for faster response. */
#define BTN_DEBOUNCE_MS      20
/* CAPTURE_MODE is defined above so CAM_BINNING sees it early. */
/** Delay between button poll iterations (milliseconds).
    Reduced from 30 to 10 for faster detection. */
#define BTN_POLL_DELAY_MS    10

/* ================================================================
   SECTION 5: LEGACY SYMBOLS (keep for compatibility with app.c)
   UVC/JPEG code still references these even in standalone mode.
   ================================================================ */

/** Legacy UVC stream table (not used in standalone, but referenced by app.c) */
#define IMG_STREAMS {{UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 640, 480, 30}}

/** Legacy max frame size (used by app.c / app_jpg.c buffer allocation).
    In STANDALONE_MODE this is only for compatibility — actual snapshot
    buffer size is controlled by MAX_SNAP_FRAME_SIZE. */
#ifndef STANDALONE_MODE
#define MAX_IMG_FRAME_SIZE (800 * 480 * 2)
#else
#define MAX_IMG_FRAME_SIZE (0)  /* UVC buffers not allocated in standalone */
#endif

/* ================================================================
   SECTION 7: PERFORMANCE DEBUG / TIMING CONTROL
   ================================================================ */

/** Enable detailed performance timing prints.
    0 = Minimal prints (production: only total time per snapshot)
    1 = Standard prints (phase breakdown: camera init, capture, SD write)
    2 = VERBOSE prints (sub-phase timing: exposure config, warmup, per-batch SD, cache ops)
    3 = DEBUG prints (everything + per-block SD timing + wait-state analysis)

    For bottleneck analysis, use level 2 or 3.
    For production deployment, use level 0.
    CAPTURE_MODE = 5 (sleep-snapshot) defaults to 0: the sleep state
    machine prints its own explicit [SLEEP]/[CAM]/[SD] messages. */
#if CAPTURE_MODE == 5
#define PERF_DEBUG_LEVEL         0
#else
#define PERF_DEBUG_LEVEL         2
#endif

/** When PERF_DEBUG_LEVEL >= 2, print SD batch timing every N batches.
    Set to 1 for every batch (very verbose), 4 for every 4th batch. */
#define PERF_SD_BATCH_PRINT_EVERY  0

/** Track cumulative waiting time for SD card ready states.
    When enabled, the final summary will show total time spent waiting
    for the card to be ready vs actual DMA transfer time. */
#define PERF_TRACK_SD_WAIT_TIME    1

/** Print a performance summary after each complete capture cycle.
    Shows: phase breakdown, bottleneck identification, throughput MB/s. */
#define PERF_PRINT_SUMMARY         1

/** Maximum number of snapshots to track for running statistics.
    Set to 0 for no stats, 10 for average over last 10 captures. */
#define PERF_STATS_WINDOW          10

/* ================================================================
   SECTION 8: WS2812 ILLUMINATION CONFIGURATION
   ================================================================ */

/** Illumination mode:
    0 = ALWAYS_ON  — LEDs stay on continuously (power hungry, not recommended)
    1 = CAPTURE    — LEDs turn on at detection, stay on during full camera
                      capture cycle (warmup + frame grab + SD write), then off.
                      Uses FlashStart/FlashStop (non-blocking).
                      This is the CORRECT mode for insect capture!
    2 = INDICATOR  — LEDs flash briefly on detection only (no illumination) */
#define WS2812_MODE                1

/** Illumination ON duration in milliseconds.
    NOTE: With FlashStart/FlashStop approach (WS2812_MODE=1), this value
    is NOT used. LEDs stay on for the exact duration of the capture cycle
    (determined by semaphores, not timers).
    Kept here for backward compatibility with WS2812_Illuminate(). */
#define WS2812_ILLUMINATION_MS     500

/** Illumination brightness (0-100%).
    During capture: LEDs run at this brightness.
    Higher = brighter image but more power consumption.
    At 100%: Maximum LED output
    At 50%:  Half brightness (may need higher camera gain to compensate)

    TUNING NOTE (2026-07-16): raised from 10% - at only 10% brightness combined
    with an ultra-short exposure, the sensor was starved of light, forcing high
    gain and visible noise that looked like blur. WS2812_MODE=1 only flashes the
    LEDs for the duration of one capture burst (not continuously), so running
    brighter here has negligible power/thermal impact. Brightness is applied as
    RGB value scaling (not time-based PWM), so it stays perfectly in sync with
    even very short camera exposures - safe to raise further if still too dark. */
#define WS2812_ILLUMINATION_BRIGHTNESS  50

/** Illumination color new update 0xGGRRBB!!!
    White (0xFFFFFF): Maximum illumination for camera — RECOMMENDED
    Green (0x00FF00): Insects less sensitive, more natural behavior
    Red (0xFF0000): Least disruptive to insects but camera needs more gain */
#define WS2812_ILLUMINATION_COLOR       0xC8B080//0xD0E654//0xA0FF28

/** Visual indicator flash (optional).
    After illumination stops, flash LEDs briefly to confirm detection.
    0 = No indicator (silent operation)
    50-200ms = Brief visual confirmation */
#define WS2812_INDICATOR_MS          0

/* ================================================================
   SECTION 9: SLEEP-SNAPSHOT MODE (CAPTURE_MODE = 5)
   MCU parks in STOP sleep; USER button (PC13 = PWR WKUP3, HIGH) wakes.
   ================================================================ */

/** Sensor standby->streaming settle time after wake (ms).
    Same proven value as the CAPTURE_MODE=4 wake path. */
#define SLEEP_SENSOR_SETTLE_MS    35

/** Grace period after SD save before re-entering STOP sleep (ms).
    Absorbs button bounce / held presses and lets the console finish. */
#define SLEEP_ENTER_DELAY_MS      500

/** Turn user LEDs (green/red) OFF while sleeping to save current. */
#define SLEEP_LEDS_OFF            1

/** Put NOR flash (XSPI2, instance 0) into deep power-down while sleeping.
    1 = yes (saves uA, default)    0 = keep NOR active (faster wake) */
#define SLEEP_NOR_DEEP_PD         1

/** Extra settle time (ms) after the last console print before WFI,
    so the UART transmitter is guaranteed idle when clocks stop
    (the console is blocking; this is only extra margin). */
#define SLEEP_UART_SETTLE_MS      10

#endif /* APP_CONFIG */
