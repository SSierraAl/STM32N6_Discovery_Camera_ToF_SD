#ifndef RAW_IMAGE_H
#define RAW_IMAGE_H
#include "ds3231.h"
#define SD_HEADER_TAG 0x49444745U
#define RAW_TIME_TAG 0x31435452U /* RTC1, little endian. */
typedef struct {
    uint32_t magic, width, height, pixel_format, data_size, timestamp, checksum, snap_id;
    uint32_t time_tag, time_flags, capture_tick;
    uint8_t reserved[20];
} sd_image_header_t;
_Static_assert(sizeof(sd_image_header_t)==64,"Raw image header must occupy 64 bytes");
static inline void RawImage_SetStamp(sd_image_header_t *h, RTC_Stamp s) {
    h->timestamp=s.unix_seconds; h->time_tag=RAW_TIME_TAG;
    h->time_flags=2U|(s.valid?1U:0U); /* Bit 1: post-capture/batch completion. */
    h->capture_tick=s.capture_tick;
}
#endif
