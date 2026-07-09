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

/** Camera binning mode — simple toggle to switch resolutions.
    0 = FULL RESOLUTION (2592x1944, 5MPX) — Default, best for species ID
    1 = 2x2 BINNING (1296x972, 1.3MPX) — Faster readout, less rolling shutter */
#define CAM_BINNING          1

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
     REDUCED FROM 11 → 4: Saves ~167ms per capture (33ms saved per frame).
     CRITICAL: Each warmup frame = ~33ms of delay where the insect can move!
     If images have green tint on first few captures, increase this value. */
#define SNAP_WARMUP_FRAMES   11

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
    Recommended: 0 (AUTO) - IMX335 auto-exposure works well */
#define CAM_EXPOSURE_MODE    1

/** Exposure time in MICROSECONDS (only used in MANUAL mode).
     The IMX335 sensor expects exposure in µs.
     Valid range: ~1000-33000 µs (depends on FPS setting).
     
     NOTE: At 30 FPS, one frame period = 33,333 µs. Exposure cannot exceed this.
     For OUTDOOR (bright light): 2000-5000 µs works well.
     
     IMPORTANT: Check console output for "[CAM] Readback: exposure=XXX"
     If readback differs from configured value, sensor driver is clamping.
     Values below ~1000 µs are often clamped to minimum by IMX335 driver.
     
     Current: 1000 µs (minimum practical for outdoor, bright conditions) */
#define CAM_EXPOSURE_VALUE   50

/** Analog gain (only used in MANUAL mode).
     Range: 0-2047. Higher = brighter image but more noise/grain.
     For OUTDOOR (bright light): Lower gain is better (less noise).
     Good starting points:
       - Outdoor bright:    200-400 (clean image, short exposure)
       - Outdoor shade:     600-1000
       - Indoor bright:     1000-1500
     
     Current: 300 (outdoor, bright conditions, minimal noise) */
#define CAM_GAIN_VALUE       200

/** Brightness adjustment.
    Range: depends on sensor (typically -128 to +127).
    0 = default. Positive = brighter, negative = darker.
    Default: 0 */
#define CAM_BRIGHTNESS       0

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
    Use smaller batches + longer waits = more reliable.

    Reduced from 128 to 64: More batches but more reliable. */
#define SD_BATCH_WRITE_BLOCKS 64

/** Maximum snapshots that can be stored before SD card overflow.
    For a 32 GB SDHC card (64,000,000 blocks):
    (64,000,000 - 3072) / 19,643 ≈ 3,257 snapshots max.
    This is a safety constant; the actual limit depends on card size. */
#define SD_MAX_SNAPSHOTS       3000

/* ================================================================
   SECTION 4: CAPTURE MODE SELECTION
   ================================================================ */

/** Capture mode for ToF-triggered photography.
    0 = ON-DEMAND  — Camera off until trigger. Full init → warmup → capture → deinit cycle.
                      Safe but slow (~400ms latency). Good for stationary/slow objects.
    1 = CONTINUOUS — Camera always running in continuous mode. On trigger: stop → copy → restart.
                      Very fast (~5-10ms latency). Good for fast-moving objects.
                      Requires extra PSRAM buffer (~5MB) for frame copy during SD write. */
#define CAPTURE_MODE             0

/* ================================================================
   SECTION 5: BUTTON / LED PARAMETERS
   ================================================================ */

/** Debounce time for USER button press detection (milliseconds).
    Reduced from 50 to 20 for faster response. */
#define BTN_DEBOUNCE_MS      20

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
    For production deployment, use level 0. */
#define PERF_DEBUG_LEVEL         1

/** When PERF_DEBUG_LEVEL >= 2, print SD batch timing every N batches.
    Set to 1 for every batch (very verbose), 4 for every 4th batch. */
#define PERF_SD_BATCH_PRINT_EVERY  4

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
                     capture cycle (warmup + frame grab), then off.
                     This is the CORRECT mode for insect capture! */
#define WS2812_MODE                1

/** Illumination ON duration in milliseconds.
    MUST be LONGER than the full camera capture cycle (warmup + frame grab).
    
    Camera timing breakdown:
      - Warmup frames: 11 × 33ms = 363ms
      - Capture frame: 1 × 33ms = 33ms
      - Total capture: ~396ms (confirmed in logs)
      - Safety margin: +50ms
    
    Recommended minimum: 450ms
    If you reduce SNAP_WARMUP_FRAMES, adjust this accordingly:
      Formula: (SNAP_WARMUP_FRAMES + 1) × (1000/SNAP_FPS) + 50ms
              = (11 + 1) × (1000/30) + 50 = 450ms */
#define WS2812_ILLUMINATION_MS     500

/** Illumination brightness (0-100%).
    During capture: LEDs run at this brightness.
    Higher = brighter image but more power consumption.
    At 100%: Maximum LED output
    At 50%:  Half brightness (may need higher camera gain to compensate) */
#define WS2812_ILLUMINATION_BRIGHTNESS  100

/** Illumination color (0xRRGGBB format).
    White (0xFFFFFF): Maximum illumination for camera — RECOMMENDED
    Green (0x00FF00): Insects less sensitive, more natural behavior
    Red (0xFF0000): Least disruptive to insects but camera needs more gain */
#define WS2812_ILLUMINATION_COLOR       0xFFFFFF

/** Visual indicator flash (optional).
    After illumination stops, flash LEDs briefly to confirm detection.
    0 = No indicator (silent operation)
    50-200ms = Brief visual confirmation */
#define WS2812_INDICATOR_MS          0

#endif /* APP_CONFIG */
