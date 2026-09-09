/*
 * Persistent raw-SD append journal shared transparently by CAPTURE_MODE 0/4.
 *
 * The existing application uses logical block addresses that restart at
 * SD_SNAP_BASE_BLOCK after every MCU reset. This shim keeps those call sites
 * unchanged and translates each image to a persistent physical append point.
 *
 * Safety policy: reserve first, write second. A power loss can leave unused or
 * partially-written blocks, but an older image is never intentionally reused.
 */
#include "stm32n6xx_hal.h"
#include "app_config.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SD_STORAGE_JOURNAL_INTERNAL 1
#include "raw_image.h"

#define SDJ_MAGIC       0x314A4453U /* "SDJ1" little endian */
#define SDJ_VERSION     1U
#define SDJ_BLOCK_A     (SD_SNAP_BASE_BLOCK - 2U)
#define SDJ_BLOCK_B     (SD_SNAP_BASE_BLOCK - 1U)
#define SDJ_SECTOR_SIZE 512U
#define SDJ_INIT_TRIES  3U
#define SDJ_INIT_RETRY_MS 150U

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t next_block;
    uint32_t next_snap_id;
    uint32_t snap_base;
    uint32_t crc32;
} SDJ_Record;

_Static_assert(sizeof(SDJ_Record) == 28U, "Unexpected SD journal layout");

static SDJ_Record g_state;
static uint8_t g_state_loaded = 0U;
static uint8_t g_state_slot = 0xFFU; /* 0=A, 1=B */
static uint8_t g_missing_reported = 0U;

/* Current logical->physical translation, valid only while one image is being
   emitted by the existing batched writer. */
static uint8_t g_map_active = 0U;
static uint32_t g_logical_start = 0U;
static uint32_t g_logical_end = 0U;
static uint32_t g_physical_start = 0U;

static uint8_t g_io_sector[SDJ_SECTOR_SIZE] __attribute__((aligned(32)));

static uint32_t sdj_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFU;
    while (len--) {
        crc ^= *p++;
        for (unsigned b = 0; b < 8U; ++b)
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

static uint32_t sdj_record_crc(const SDJ_Record *r)
{
    return sdj_crc32(r, offsetof(SDJ_Record, crc32));
}

static int sdj_wait_ready(SD_HandleTypeDef *hsd, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - start) >= timeout_ms)
            return -1;
        HAL_Delay(1);
    }
    return 0;
}

static int sdj_record_valid(const SDJ_Record *r, const SD_HandleTypeDef *hsd)
{
    if (!r || !hsd) return 0;
    if (r->magic != SDJ_MAGIC || r->version != SDJ_VERSION) return 0;
    if (r->snap_base != SD_SNAP_BASE_BLOCK) return 0;
    if (r->next_block < SD_SNAP_BASE_BLOCK) return 0;
    if (hsd->SdCard.BlockNbr != 0U && r->next_block > hsd->SdCard.BlockNbr) return 0;
    return r->crc32 == sdj_record_crc(r);
}

static int sdj_read_record(SD_HandleTypeDef *hsd, uint32_t block, SDJ_Record *out)
{
    if (sdj_wait_ready(hsd, 2000U) != 0) return -1;
    memset(g_io_sector, 0, sizeof(g_io_sector));
    SCB_InvalidateDCache_by_Addr((uint32_t *)g_io_sector, sizeof(g_io_sector));
    if (HAL_SD_ReadBlocks(hsd, g_io_sector, block, 1U, HAL_MAX_DELAY) != HAL_OK)
        return -1;
    SCB_InvalidateDCache_by_Addr((uint32_t *)g_io_sector, sizeof(g_io_sector));
    memcpy(out, g_io_sector, sizeof(*out));
    return 0;
}

static int sdj_write_record(SD_HandleTypeDef *hsd, uint32_t block, SDJ_Record *r)
{
    r->crc32 = sdj_record_crc(r);
    if (sdj_wait_ready(hsd, 2000U) != 0) return -1;

    memset(g_io_sector, 0, sizeof(g_io_sector));
    memcpy(g_io_sector, r, sizeof(*r));
    SCB_CleanDCache_by_Addr((uint32_t *)g_io_sector, sizeof(g_io_sector));
    if (HAL_SD_WriteBlocks(hsd, g_io_sector, block, 1U, HAL_MAX_DELAY) != HAL_OK)
        return -1;
    if (sdj_wait_ready(hsd, 2000U) != 0) return -1;

    /* Read-after-write verification: a torn/corrupt metadata write must never
       become the authority for a future boot. */
    SDJ_Record verify;
    if (sdj_read_record(hsd, block, &verify) != 0) return -1;
    if (!sdj_record_valid(&verify, hsd)) return -1;
    if (memcmp(&verify, r, sizeof(*r)) != 0) return -1;
    return 0;
}

static int sdj_load(SD_HandleTypeDef *hsd)
{
    if (g_state_loaded) return 0;

    SDJ_Record a = {0}, b = {0};
    int va = (sdj_read_record(hsd, SDJ_BLOCK_A, &a) == 0) && sdj_record_valid(&a, hsd);
    int vb = (sdj_read_record(hsd, SDJ_BLOCK_B, &b) == 0) && sdj_record_valid(&b, hsd);

    if (!va && !vb) {
        if (!g_missing_reported) {
            printf("[SD META] No valid append journal at blocks %lu/%lu.\n",
                   (unsigned long)SDJ_BLOCK_A, (unsigned long)SDJ_BLOCK_B);
            printf("[SD META] Image writes BLOCKED to avoid overwriting legacy photos. "
                   "Initialize/repair this card with SD_Storage_Journal.py.\n");
            g_missing_reported = 1U;
        }
        return -1;
    }

    if (va && (!vb || (int32_t)(a.sequence - b.sequence) > 0)) {
        g_state = a;
        g_state_slot = 0U;
    } else {
        g_state = b;
        g_state_slot = 1U;
    }
    g_state_loaded = 1U;
    g_missing_reported = 0U;
    printf("[SD META] Loaded seq=%lu next_block=%lu next_id=%lu\n",
           (unsigned long)g_state.sequence,
           (unsigned long)g_state.next_block,
           (unsigned long)g_state.next_snap_id);
    return 0;
}

static int sdj_reserve_image(SD_HandleTypeDef *hsd, uint32_t image_blocks,
                             uint32_t *physical_start, uint32_t *snap_id)
{
    if (sdj_load(hsd) != 0) return -1;
    if (image_blocks == 0U) return -1;

    uint32_t start = g_state.next_block;
    uint64_t end64 = (uint64_t)start + (uint64_t)image_blocks;
    if (end64 > 0xFFFFFFFFULL) return -1;
    uint32_t end = (uint32_t)end64;
    if (hsd->SdCard.BlockNbr != 0U && end > hsd->SdCard.BlockNbr) {
        printf("[SD META] Card full: need blocks %lu..%lu, card has %lu\n",
               (unsigned long)start, (unsigned long)(end - 1U),
               (unsigned long)hsd->SdCard.BlockNbr);
        return -1;
    }

    SDJ_Record next = g_state;
    next.sequence = g_state.sequence + 1U;
    next.next_block = end;
    next.next_snap_id = g_state.next_snap_id + 1U;
    next.snap_base = SD_SNAP_BASE_BLOCK;
    next.magic = SDJ_MAGIC;
    next.version = SDJ_VERSION;

    uint8_t target_slot = (g_state_slot == 0U) ? 1U : 0U;
    uint32_t target_block = target_slot ? SDJ_BLOCK_B : SDJ_BLOCK_A;
    if (sdj_write_record(hsd, target_block, &next) != 0) {
        printf("[SD META] Reserve FAILED; image write cancelled before touching image area\n");
        return -1;
    }

    *physical_start = start;
    *snap_id = g_state.next_snap_id;
    g_state = next;
    g_state_slot = target_slot;
    return 0;
}

static int sdj_is_image_header(const uint8_t *data, uint32_t blocks,
                               uint32_t *image_blocks)
{
    if (!data || blocks == 0U) return 0;
    const sd_image_header_t *h = (const sd_image_header_t *)data;
    if (h->magic != SD_HEADER_TAG) return 0;
    if (h->width == 0U || h->height == 0U || h->width > 4096U || h->height > 4096U)
        return 0;
    uint32_t bpp = (h->pixel_format == 2U) ? 1U : 2U;
    uint64_t expected = (uint64_t)h->width * (uint64_t)h->height * bpp;
    if (expected == 0U || expected > 0xFFFFFFFFULL || h->data_size != (uint32_t)expected)
        return 0;
    *image_blocks = (64U + h->data_size + 511U) / 512U;
    return *image_blocks > 0U;
}

/* Replacement name is injected by raw_image.h before the HAL prototype is
   parsed in main.c/app_thread.c. The real HAL symbol is visible in this file. */
HAL_StatusTypeDef SDJ_HAL_SD_Init(SD_HandleTypeDef *hsd)
{
    if (!hsd) return HAL_ERROR;

    /* Any init can correspond to a removed/reinserted or completely different
       card, so never carry an old card's mapping across it. */
    g_state_loaded = 0U;
    g_state_slot = 0xFFU;
    g_missing_reported = 0U;
    g_map_active = 0U;

    /* Boot already uses ClockDiv=4. Force the same validated value for the
       recovery path, which previously changed to ClockDiv=2. */
    hsd->Init.ClockDiv = 4U;

    HAL_StatusTypeDef st = HAL_ERROR;
    for (uint32_t attempt = 0U; attempt < SDJ_INIT_TRIES; ++attempt) {
        st = HAL_SD_Init(hsd);
        if (st == HAL_OK) return HAL_OK;
        if (attempt + 1U >= SDJ_INIT_TRIES) break;

        printf("[SD] Init retry %lu/%lu after HAL=%lu\n",
               (unsigned long)(attempt + 2U), (unsigned long)SDJ_INIT_TRIES,
               (unsigned long)st);
        (void)HAL_SD_DeInit(hsd);
        __HAL_RCC_SDMMC2_FORCE_RESET();
        HAL_Delay(10U);
        __HAL_RCC_SDMMC2_RELEASE_RESET();
        HAL_Delay(SDJ_INIT_RETRY_MS);
        hsd->State = HAL_SD_STATE_RESET;
        HAL_SD_MspInit(hsd);
    }
    return st;
}

HAL_StatusTypeDef SDJ_HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, const uint8_t *pData,
                                         uint32_t BlockAdd, uint32_t NumberOfBlocks,
                                         uint32_t Timeout)
{
    if (!hsd || !pData || NumberOfBlocks == 0U) return HAL_ERROR;

    /* Benchmark, FAT/utility writes and anything before the raw photo area are
       deliberately untouched. */
    if (BlockAdd < SD_SNAP_BASE_BLOCK)
        return HAL_SD_WriteBlocks(hsd, pData, BlockAdd, NumberOfBlocks, Timeout);

    uint32_t image_blocks = 0U;
    if (sdj_is_image_header(pData, NumberOfBlocks, &image_blocks)) {
        uint32_t physical_start = 0U, persistent_id = 0U;
        if (sdj_reserve_image(hsd, image_blocks, &physical_start, &persistent_id) != 0)
            return HAL_ERROR;

        /* All current image writers use mutable sd_batch_buf. Persist the ID
           owned by the card, not the volatile counter that resets on MCU boot. */
        sd_image_header_t *h = (sd_image_header_t *)(uintptr_t)pData;
        h->snap_id = persistent_id;
        SCB_CleanDCache_by_Addr((uint32_t *)(uintptr_t)pData,
                                NumberOfBlocks * SD_BLOCK_SIZE);

        g_map_active = 1U;
        g_logical_start = BlockAdd;
        g_logical_end = BlockAdd + image_blocks;
        g_physical_start = physical_start;

        printf("[SD META] Reserve id=%lu logical=%lu physical=%lu next=%lu\n",
               (unsigned long)persistent_id, (unsigned long)BlockAdd,
               (unsigned long)physical_start, (unsigned long)g_state.next_block);
    }

    if (!g_map_active || BlockAdd < g_logical_start || BlockAdd >= g_logical_end) {
        printf("[SD META] Refusing untracked write at image block %lu\n",
               (unsigned long)BlockAdd);
        return HAL_ERROR;
    }

    uint32_t physical = g_physical_start + (BlockAdd - g_logical_start);
    uint64_t physical_end = (uint64_t)physical + NumberOfBlocks;
    uint64_t mapped_end = (uint64_t)g_physical_start +
                          (uint64_t)(g_logical_end - g_logical_start);
    if (physical_end > mapped_end) {
        printf("[SD META] Refusing write beyond reserved image range\n");
        return HAL_ERROR;
    }

    HAL_StatusTypeDef st = HAL_SD_WriteBlocks(hsd, pData, physical, NumberOfBlocks, Timeout);
    if (st == HAL_OK && (BlockAdd + NumberOfBlocks) >= g_logical_end)
        g_map_active = 0U;
    return st;
}
