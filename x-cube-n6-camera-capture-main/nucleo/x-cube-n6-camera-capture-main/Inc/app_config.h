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

/** Snapshot resolution (width)  --  YUV422 format = 2 bytes/pixel */
#define SNAP_WIDTH          2592

/** Snapshot resolution (height) */
#define SNAP_HEIGHT          1944

/** Camera frame rate (FPS).
    Valid values depend on resolution:
      - 640x480  : 30 or 60 FPS
      - 1024x768 : 30 FPS only
      - 1920x1080: 15 or 30 FPS
      - 2592 x 1944 max */
#define SNAP_FPS             30

/** Number of warmup frames to discard after camera power-on.
    The IMX335 sensor outputs garbage on the first frames.
    Reduced from 11 to 8 to save ~100ms per capture (33ms saved per frame).
    If images have green tint, increase this value. */
#define SNAP_WARMUP_FRAMES   11

/** Maximum time to wait for warmup + capture frames (milliseconds). */
#define SNAP_TIMEOUT_MS      400

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
    The IMX335 sensor expects exposure in µs, NOT 0-200 arbitrary units!
    Valid range: ~1000-33000 µs (depends on FPS setting).
    
    Good starting points:
      - Fast moving objects (bright): 1000-2000 µs + high gain
      - Fast moving objects (indoor): 2000-4000 µs + high gain  
      - Normal indoor:                 5000-10000 µs + moderate gain
      - Dim lighting:                  15000-30000 µs + low gain
    
    Default: 3000 µs (good balance for moving objects indoors) */
#define CAM_EXPOSURE_VALUE   1000

/** Analog gain (only used in MANUAL mode).
    Range: 0-2047. Higher = brighter image but more noise/grain.
    For short exposure (3000µs), you need high gain to compensate.
    Good starting points:
      - With exposure=3000µs: 1000-1500 (bright but noisy)
      - With exposure=5000µs: 600-1000 (good balance) 
      - With exposure=10000µs: 300-600 (cleaner image)
      - Dim lighting long exp: 100-300 (minimize noise)
    Default: 1200 (high gain for 3000µs exposure) */
#define CAM_GAIN_VALUE       600

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
    Larger values reduce the number of HAL API calls (less overhead = faster).
    - 1  block/call  = 19,643 calls per snapshot (original, slow)
    - 64 blocks/call = ~307 calls per snapshot (64x fewer calls)
    - 128 blocks/call = ~154 calls per snapshot (128x fewer calls)

    Buffer size in PSRAM = SD_BATCH_WRITE_BLOCKS * 512 bytes.
    With 64 blocks: 32 KB buffer. With 128 blocks: 64 KB buffer.
    PSRAM has ~6.4 MB free after capture_buf, so even 128 is safe. */
#define SD_BATCH_WRITE_BLOCKS  64

/** Maximum snapshots that can be stored before SD card overflow.
    For a 32 GB SDHC card (64,000,000 blocks):
    (64,000,000 - 3072) / 19,643 ≈ 3,257 snapshots max.
    This is a safety constant; the actual limit depends on card size. */
#define SD_MAX_SNAPSHOTS       3000

/* ================================================================
   SECTION 4: BUTTON / LED PARAMETERS
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

#endif /* APP_CONFIG */
