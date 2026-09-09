#ifndef RAW_IMAGE_H
#define RAW_IMAGE_H
#include "ds3231.h"

/*
 * SD persistence shim.
 *
 * main.c (Mode 0) and app_thread.c (Mode 4) both include this header before
 * the STM32 HAL headers. Renaming only these two HAL entry points lets the
 * journal layer keep the existing capture/storage code untouched:
 *   - SDJ_HAL_SD_Init() resets per-card journal state and uses the validated
 *     ClockDiv=4 setting after a card is reinserted.
 *   - SDJ_HAL_SD_WriteBlocks() passes ordinary writes through unchanged, but
 *     remaps valid raw-image writes to the persistent append position.
 *
 * sd_storage_journal.c defines SD_STORAGE_JOURNAL_INTERNAL so it can call the
 * real HAL functions without recursively entering the shim.
 */
#ifndef SD_STORAGE_JOURNAL_INTERNAL
#define HAL_SD_Init        SDJ_HAL_SD_Init
#define HAL_SD_WriteBlocks SDJ_HAL_SD_WriteBlocks
#endif

#define SD_HEADER_TAG 0x49444745U
#define RAW_TIME_TAG 0x31435452U /* RTC1, little endian. */
#define RAW_TIME_VALID            0x01U
#define RAW_TIME_RTC_POST_CAPTURE 0x02U
typedef struct {
    uint32_t magic, width, height, pixel_format, data_size, timestamp, checksum, snap_id;
    uint32_t time_tag, time_flags, capture_tick;
    uint8_t reserved[20];
} sd_image_header_t;
_Static_assert(sizeof(sd_image_header_t)==64,"Raw image header must occupy 64 bytes");
static inline void RawImage_SetStamp(sd_image_header_t *h, RTC_Stamp s) {
    h->timestamp=s.unix_seconds;
    h->time_tag=RAW_TIME_TAG;
    /* UTC comes from a post-capture RTC sample. In Mode 4 capture_tick is
       the individual frame-completion tick, so the four frames keep their
       real sub-second ordering without changing the 64-byte SD format. */
    h->time_flags=RAW_TIME_RTC_POST_CAPTURE|(s.valid?RAW_TIME_VALID:0U);
    h->capture_tick=s.capture_tick;
}
#endif
