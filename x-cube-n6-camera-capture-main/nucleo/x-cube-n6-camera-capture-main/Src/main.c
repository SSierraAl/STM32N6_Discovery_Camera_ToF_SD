/**
 * *****************************************************************************
 * @file    main.c
 * @brief   Insect Detection + Auto-Capture — NUCLEO-N657X0-Q
 *
 *   VL53L5CX Time-of-Flight sensor monitors for insect passage using per-zone
 *   signal-drop detection. When an insect is detected, the camera auto-captures
 *   a YUV422 frame and saves it directly to microSD card.
 *
 *   No USB/UVC, no FileX/fatfs, no PC connection required after flashing.
 *
 *   Hardware:
 *     - MCU:      STM32N657 (NUCLEO-N657X0-Q / MB1940)
 *     - Camera:   CMW-IMX335 (5 MP, YUV422 output via CSI-2)
 *     - ToF:      VL53L5CX (4x4 resolution, continuous mode)
 *     - Storage:  microSD card (SDMMC2, 4-bit bus)
 *     - LEDs:     RED = PG10, GREEN = PG0
 *     - Illumination: WS2812 (GPDMA1 + TIM1 PWM)
 *
 *   Flow:
 *     1. Boot → init clocks, MPU, PSRAM, SD card
 *     2. GREEN LED on (system ready)
 *     3. ToF sensor learns baseline (10 samples)
 *     4. Monitoring loop watches 16 zones for signal drop
 *     5. Insect detected → flash WS2812 → capture → save to SD → cooldown
 *     6. Repeat monitoring
 *
 *   Two FreeRTOS tasks:
 *     - main_thread:     board init, SD card init, then deletes itself
 *     - insect_task:     ToF monitoring + auto-capture (runs forever)
 *
 *   All configurable parameters are in Inc/app_config.h.
 * *****************************************************************************
 */

/* ================================================================
   SECTION 1: INCLUDES
   ================================================================ */

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* ---- STM32 HAL / BSP ---- */
#include "stm32n6xx_hal_conf.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_sd.h"
#include "stm32n6xx_hal_rif.h"

#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_bus.h"
#include "stm32n6570_discovery_xspi.h"

/* ---- Illumination (WS2812) ---- */
#include "ws2812.h"

/* ---- Time of Flight (VL53L5CX) ---- */
#include "platform.h"
#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_motion_indicator.h"

/* ---- Application Modules ---- */
#include "app_config.h"
#include "app_cam.h"
#include "app_fuseprogramming.h"
#include "main.h"
#include "npu_cache.h"
#include "utils.h"

/* ---- FreeRTOS ---- */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ================================================================
   SECTION 2: GLOBAL PERIPHERAL HANDLES
   ================================================================ */

/* I2C */
I2C_HandleTypeDef hi2c1;

/* UART (Debug Console) */
UART_HandleTypeDef huart1;

/* microSD (SDMMC2) */
SD_HandleTypeDef hsd1;

/* TIM1 (WS2812 PWM) */
TIM_HandleTypeDef htim1;

/* DMA (WS2812 Data) */
DMA_HandleTypeDef handle_GPDMA1_Channel1;

/* VL53L5CX ToF Sensor State */
VL53L5CX_Configuration  Dev;
VL53L5CX_ResultsData    Results;
uint8_t                 status;
uint8_t                 isAlive;
uint8_t                 isReady;

/* ---- Insect detection parameters ---- */
#define BASELINE5CX_SAMPLES      10
#define INSECT5CX_THRESHOLD_PCT  10
#define NUM_ZONES                16
#define MOTION_INDICATOR_THRESH  20
#define CAPTURE_COOLDOWN_FRAMES  60   /* ~4 seconds at 15 Hz */

static uint32_t  g_baseline_signal[NUM_ZONES] = {0};
static uint16_t  g_baseline_distance[NUM_ZONES] = {0};
static uint8_t   g_zone_valid[NUM_ZONES] = {0};
static uint8_t   g_baseline_ready = 0;

/* ================================================================
   SECTION 3: FREERTOS TASK CONTROL BLOCKS AND STACKS
   ================================================================ */

static StaticTask_t    main_thread_cb;
static StackType_t     main_thread_stack[configMINIMAL_STACK_SIZE];

static StaticTask_t    insect_thread_cb;
static StackType_t     insect_thread_stack[8 * configMINIMAL_STACK_SIZE];

/* ================================================================
   SECTION 4: FORWARD DECLARATIONS
   ================================================================ */

/* ---- System Configuration ---- */
static void SystemClock_Config(void);
static void Security_Config(void);
static void IAC_Config(void);
static void CONSOLE_Config(void);
static void Setup_Mpu(void);

/* ---- FreeRTOS Entry Points ---- */
static int   main_freertos(void);
static void  main_thread_fct(void *arg);
static void  insect_detection_task(void *arg);

/* ---- SD Card Operations ---- */
static void  SD_Benchmarks(void);
static int   SD_StoreRawImage(const uint8_t *img_buf, uint32_t img_size,
                              uint32_t w, uint32_t h, uint32_t pixel_format);
static int   SD_ReadRawImage(uint8_t *img_buf, uint32_t *data_size_p);

/* ---- Peripheral Initialization (WS2812 / Illumination) ---- */
static void MX_TIM1_Init(void);
static void MX_GPDMA1_Init(void);
void        PeriphCommonClock_Config(void);

/* ---- Peripheral Initialization (VL53L5CX ToF Sensor) ---- */
static void VL53L5CX_I2C_Init(void);
static void VL53L5CX_GPIO_Init(void);
static void VL53L5CX_StartSequence(void);
static void VL53L5CX_Validate(void);
static void VL53L5CX_ReadingTest(void);
static void VL53L5CX_MotionTest(void);

/* ---- External References ---- */
extern void  vPortSetupTimerInterrupt(void);
extern int   __uncached_bss_start__;
extern int   __uncached_bss_end__;

/* ================================================================
   SECTION 5: RUNTIME STATE AND BUFFERS
   ================================================================ */

static volatile int system_ready = 0;

/* Global snapshot counter — increments after each successful SD write.
   Used to compute unique SD block offsets so images never overwrite each other. */
static uint32_t snap_count      = 0;
static uint32_t snap_base_block = SD_SNAP_BASE_BLOCK;

/** Camera frame buffer — allocated in PSRAM for DMA access */
static uint8_t capture_buf[MAX_SNAP_FRAME_SIZE] ALIGN_32 IN_PSRAM;

/** SD batch-write buffer — allocated in PSRAM for multi-block writes */
static uint8_t sd_batch_buf[SD_BATCH_WRITE_BLOCKS * SD_BLOCK_SIZE] ALIGN_32 IN_PSRAM;

/* ================================================================
   SD CARD IMAGE HEADER FORMAT
   ================================================================ */

typedef struct {
    uint32_t magic;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint32_t data_size;
    uint32_t timestamp;
    uint32_t checksum;
    uint32_t snap_id;
    uint8_t  reserved[28];
} sd_image_header_t;

#define SD_HEADER_TAG       0x49444745U

/* SD_IMG_BASE_BLOCK is updated per-capture to ensure each image goes to a unique block */
static uint32_t SD_IMG_BASE_BLOCK = SD_SNAP_BASE_BLOCK;

/* ================================================================
   SECTION 6: ENTRY POINT
   ================================================================ */

int main(void)
{
    /* Enable instruction cache */
    MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_ICACTIVE_Msk;
    __HAL_RCC_CPUCLK_CONFIG(RCC_CPUCLKSOURCE_HSI);
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);

    /* Basic HAL initialization */
    HAL_Init();
    Setup_Mpu();
    SCB_EnableICache();

#if defined(USE_DCACHE)
    MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_DCACTIVE_Msk;
    SCB_EnableDCache();
#endif

    /* Start FreeRTOS scheduler */
    return main_freertos();
}

/* ================================================================
   SECTION 7: MPU SETUP
   ================================================================ */

static void Setup_Mpu(void)
{
    MPU_Attributes_InitTypeDef attr;
    MPU_Region_InitTypeDef     region;

    attr.Number     = MPU_ATTRIBUTES_NUMBER0;
    attr.Attributes = MPU_NOT_CACHEABLE;
    HAL_MPU_ConfigMemoryAttributes(&attr);

    region.Enable           = MPU_REGION_ENABLE;
    region.Number           = MPU_REGION_NUMBER0;
    region.BaseAddress      = (uint32_t)&__uncached_bss_start__;
    region.LimitAddress     = (uint32_t)&__uncached_bss_end__ - 1;
    region.AttributesIndex  = MPU_ATTRIBUTES_NUMBER0;
    region.AccessPermission = MPU_REGION_ALL_RW;
    region.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    region.DisablePrivExec  = MPU_PRIV_INSTRUCTION_ACCESS_ENABLE;
    region.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    HAL_MPU_ConfigRegion(&region);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    memset(&__uncached_bss_start__, 0, &__uncached_bss_end__ - &__uncached_bss_start__);
}

/* ================================================================
   SECTION 8: SECURITY CONFIGURATION
   ================================================================ */

static void Security_Config(void)
{
    __HAL_RCC_RIFSC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    volatile uint32_t delay = 1000;
    while (delay--);

    RIMC_MasterConfig_t RIMC_master = {0};
    RIMC_master.MasterCID = RIF_CID_1;
    RIMC_master.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;

    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU,    &RIMC_master);
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DMA2D,  &RIMC_master);
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1,  &RIMC_master);
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC2,  &RIMC_master);
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_OTG1,   &RIMC_master);

    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU,    RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DMA2D,  RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_CSI,    RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDC,   RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL2, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_JPEG,   RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_OTG1HS, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1,
                                        DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV |
                                        DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC) != HAL_OK) {
        //Error_Handler();
    }
    HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_9, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
    HAL_GPIO_ConfigPinAttributes(GPIOC, GPIO_PIN_1, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
    HAL_GPIO_ConfigPinAttributes(GPIOH, GPIO_PIN_9, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
}

/* ================================================================
   SECTION 9: IAC CONFIGURATION
   ================================================================ */

static void IAC_Config(void)
{
    __HAL_RCC_IAC_CLK_ENABLE();
    __HAL_RCC_IAC_FORCE_RESET();
    __HAL_RCC_IAC_RELEASE_RESET();
}

void IAC_IRQHandler(void) { while (1) {} }

/* ================================================================
   SECTION 10: SYSTEM CLOCK CONFIGURATION
   ================================================================ */

static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef              RCC_ClkInitStruct      = {0};
    RCC_OscInitTypeDef              RCC_OscInitStruct      = {0};
    RCC_PeriphCLKInitTypeDef        RCC_PeriphCLKInitStruct = {0};

    BSP_SMPS_Init(SMPS_VOLTAGE_OVERDRIVE);
    HAL_Delay(1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    RCC_OscInitStruct.PLL1.PLLState  = RCC_PLL_ON;
    RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL1.PLLM      = 2;
    RCC_OscInitStruct.PLL1.PLLN      = 25;
    RCC_OscInitStruct.PLL1.PLLFractional = 0;
    RCC_OscInitStruct.PLL1.PLLP1     = 1;
    RCC_OscInitStruct.PLL1.PLLP2     = 1;

    RCC_OscInitStruct.PLL2.PLLState  = RCC_PLL_ON;
    RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL2.PLLM      = 8;
    RCC_OscInitStruct.PLL2.PLLN      = 125;
    RCC_OscInitStruct.PLL2.PLLFractional = 0;
    RCC_OscInitStruct.PLL2.PLLP1     = 1;
    RCC_OscInitStruct.PLL2.PLLP2     = 1;

    RCC_OscInitStruct.PLL3.PLLState  = RCC_PLL_ON;
    RCC_OscInitStruct.PLL3.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL3.PLLM      = 16;
    RCC_OscInitStruct.PLL3.PLLN      = 225;
    RCC_OscInitStruct.PLL3.PLLFractional = 0;
    RCC_OscInitStruct.PLL3.PLLP1     = 1;
    RCC_OscInitStruct.PLL3.PLLP2     = 1;

    RCC_OscInitStruct.PLL4.PLLState  = RCC_PLL_ON;
    RCC_OscInitStruct.PLL4.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL4.PLLM      = 32;
    RCC_OscInitStruct.PLL4.PLLN      = 20;
    RCC_OscInitStruct.PLL4.PLLFractional = 0;
    RCC_OscInitStruct.PLL4.PLLP1     = 1;
    RCC_OscInitStruct.PLL4.PLLP2     = 1;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while (1); }

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_PCLK1 |
                                   RCC_CLOCKTYPE_PCLK2   | RCC_CLOCKTYPE_PCLK4 |
                                   RCC_CLOCKTYPE_PCLK5);
    RCC_ClkInitStruct.CPUCLKSource   = RCC_CPUCLKSOURCE_IC1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
    RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
    RCC_ClkInitStruct.IC1Selection.ClockDivider   = 1;
    RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
    RCC_ClkInitStruct.IC2Selection.ClockDivider   = 2;
    RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL2;
    RCC_ClkInitStruct.IC6Selection.ClockDivider   = 1;
    RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL3;
    RCC_ClkInitStruct.IC11Selection.ClockDivider   = 1;
    RCC_ClkInitStruct.AHBCLKDivider   = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider  = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider  = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider  = RCC_APB4_DIV1;
    RCC_ClkInitStruct.APB5CLKDivider  = RCC_APB5_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) { while (1); }

    RCC_PeriphCLKInitStruct.PeriphClockSelection = 0;
    RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_XSPI1;
    RCC_PeriphCLKInitStruct.Xspi1ClockSelection  = RCC_XSPI1CLKSOURCE_HCLK;
    RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_XSPI2;
    RCC_PeriphCLKInitStruct.Xspi2ClockSelection  = RCC_XSPI2CLKSOURCE_HCLK;
    RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_SDMMC2;
    RCC_PeriphCLKInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_HCLK;

    if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK) { while (1); }
}

/* ================================================================
   SECTION 11: DEBUG CONSOLE (UART)
   ================================================================ */

static void CONSOLE_Config(void)
{
    GPIO_InitTypeDef gpio_init;
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio_init.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOE, &gpio_init);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_8;

    if (HAL_UART_Init(&huart1) != HAL_OK) { while (1); }
}

/* ================================================================
   SECTION 12: SD CARD BENCHMARK
   ================================================================ */

#define SD_BENCH_BLOCKS  64
#define SD_BENCH_BYTES   (SD_BENCH_BLOCKS * SD_BLOCK_SIZE)

static uint8_t sd_wbuf[SD_BENCH_BYTES];
static uint8_t sd_rbuf[SD_BENCH_BYTES];

static void SD_Benchmarks(void)
{
    uint32_t          t0, t1;
    HAL_StatusTypeDef st;
    float             speed;
    uint32_t          blk = 200;
    uint32_t          i;

    for (i = 0; i < SD_BENCH_BYTES; i++)
        sd_wbuf[i] = (uint8_t)((i ^ 0xA5) & 0xFF);

    printf("\n ==== SD Speed Test (blocking, %d blocks) ====\n", SD_BENCH_BLOCKS);
    printf("  Bus : %s | ClkDiv : %lu\n\n",
           (hsd1.Init.BusWide == SDMMC_BUS_WIDE_4B) ? "4B" : "1B",
           (unsigned long)hsd1.Init.ClockDiv);

    printf("[W] ");
    t0 = HAL_GetTick();
    st = HAL_SD_WriteBlocks(&hsd1, sd_wbuf, blk, SD_BENCH_BLOCKS, HAL_MAX_DELAY);
    t1 = HAL_GetTick();

    if (st == HAL_OK && t1 > t0) {
        speed = (float)SD_BENCH_BYTES / ((float)(t1 - t0) * 1000.0f);
        printf("OK  %.2f MB/s  (%lu ms)\n", speed, (unsigned long)(t1 - t0));
    } else {
        printf("FAIL HAL=0x%08lX STA=0x%08lX\n",
               (unsigned long)st, (unsigned long)SDMMC2->STA);
        return;
    }

    printf("[R] ");
    t0 = HAL_GetTick();
    st = HAL_SD_ReadBlocks(&hsd1, sd_rbuf, blk, SD_BENCH_BLOCKS, HAL_MAX_DELAY);
    t1 = HAL_GetTick();

    if (st == HAL_OK && t1 > t0) {
        speed = (float)SD_BENCH_BYTES / ((float)(t1 - t0) * 1000.0f);
        printf("OK  %.2f MB/s  (%lu ms)\n", speed, (unsigned long)(t1 - t0));
        uint32_t errs = 0;
        for (i = 0; i < SD_BENCH_BYTES; i++)
            if (sd_rbuf[i] != sd_wbuf[i]) errs++;
        printf("[.] Integrity: %s  (%lu errors)\n",
               errs == 0 ? "PASSED" : "FAILED", errs);
    } else {
        printf("FAIL HAL=0x%08lX STA=0x%08lX\n",
               (unsigned long)st, (unsigned long)SDMMC2->STA);
    }

    printf(" ==== Done ====\n");
}

/* ================================================================
   SECTION 13: SD CARD IMAGE STORAGE (BATCHED)
   ================================================================ */

static int SD_StoreRawImage(const uint8_t *img_buf, uint32_t img_size,
                            uint32_t w, uint32_t h, uint32_t pixel_format)
{
    uint32_t         base = SD_IMG_BASE_BLOCK;
    uint32_t         total_blocks = (SD_IMG_HEADER_SIZE + img_size + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
    uint32_t         i, checksum = 0;

    if (hsd1.SdCard.BlockNbr > 0) {
        uint32_t last_block = base + total_blocks - 1;
        if (last_block >= hsd1.SdCard.BlockNbr) {
            printf("[SD Store] OVERFLOW! block %lu >= card max %lu\n",
                   (unsigned long)last_block, (unsigned long)hsd1.SdCard.BlockNbr);
            return -1;
        }
    }

    for (i = 0; i < img_size; i++)
        checksum ^= img_buf[i];

    sd_image_header_t hdr;
    hdr.magic        = SD_HEADER_TAG;
    hdr.width        = w;
    hdr.height       = h;
    hdr.pixel_format = pixel_format;
    hdr.data_size    = img_size;
    hdr.timestamp    = HAL_GetTick();
    hdr.checksum     = checksum;
    hdr.snap_id      = snap_count;  /* Store capture index in header */
    memset(hdr.reserved, 0, sizeof(hdr.reserved));

    uint32_t batch_size = SD_BATCH_WRITE_BLOCKS * SD_BLOCK_SIZE;

    printf("\n[SD Store] snap_id=%lu  %lux%lu fmt=%lu  %lu bytes  %lu blocks\n",
           (unsigned long)snap_count, (unsigned long)w, (unsigned long)h, pixel_format,
           img_size, total_blocks);

    memset(sd_batch_buf, 0, batch_size);
    memcpy(sd_batch_buf, &hdr, SD_IMG_HEADER_SIZE);
    uint32_t first_chunk = SD_BLOCK_SIZE - SD_IMG_HEADER_SIZE;
    if (first_chunk > img_size) first_chunk = img_size;
    memcpy(sd_batch_buf + SD_IMG_HEADER_SIZE, img_buf, first_chunk);

    uint32_t img_offset = first_chunk;
    uint32_t batch_blk  = 1;
    while (img_offset < img_size && batch_blk < SD_BATCH_WRITE_BLOCKS) {
        uint32_t chunk = SD_BLOCK_SIZE;
        if (chunk > img_size - img_offset) chunk = img_size - img_offset;
        memcpy(sd_batch_buf + batch_blk * SD_BLOCK_SIZE, img_buf + img_offset, chunk);
        if (chunk < SD_BLOCK_SIZE)
            memset(sd_batch_buf + batch_blk * SD_BLOCK_SIZE + chunk, 0, SD_BLOCK_SIZE - chunk);
        img_offset += chunk;
        batch_blk++;
    }

    uint32_t blocks_to_write = batch_blk;

    HAL_StatusTypeDef st = HAL_SD_WriteBlocks(&hsd1, sd_batch_buf, base, blocks_to_write, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        printf("[SD Store] header batch FAIL (blocks %lu..%lu)\n",
               (unsigned long)base, (unsigned long)(base + blocks_to_write - 1));
        return -1;
    }

    uint32_t current_block = base + blocks_to_write;

    while (img_offset < img_size) {
        uint32_t remaining_bytes = img_size - img_offset;
        uint32_t remaining_blocks_full = (remaining_bytes + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
        uint32_t blocks_in_batch = (remaining_blocks_full < SD_BATCH_WRITE_BLOCKS)
                                    ? remaining_blocks_full : SD_BATCH_WRITE_BLOCKS;

        for (uint32_t b = 0; b < blocks_in_batch; b++) {
            uint32_t src = img_offset + b * SD_BLOCK_SIZE;
            uint32_t dst = b * SD_BLOCK_SIZE;
            if (src + SD_BLOCK_SIZE <= img_size) {
                memcpy(sd_batch_buf + dst, img_buf + src, SD_BLOCK_SIZE);
            } else {
                uint32_t partial = img_size - src;
                memcpy(sd_batch_buf + dst, img_buf + src, partial);
                memset(sd_batch_buf + dst + partial, 0, SD_BLOCK_SIZE - partial);
            }
        }

        {
            uint32_t wait_ms = 0;
            while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER && wait_ms < 2000) {
                vTaskDelay(pdMS_TO_TICKS(1));
                wait_ms++;
            }
            if (wait_ms >= 2000) {
                printf("[SD Store] Card timeout before block %lu\n", (unsigned long)current_block);
            }
        }

        st = HAL_SD_WriteBlocks(&hsd1, sd_batch_buf, current_block, blocks_in_batch, HAL_MAX_DELAY);
        if (st != HAL_OK) {
            printf("[SD Store] batch FAIL at block %lu (HAL=0x%08lX STA=0x%08lX)\n",
                   (unsigned long)current_block, (unsigned long)st, (unsigned long)SDMMC2->STA);
            return -1;
        }

        img_offset += blocks_in_batch * SD_BLOCK_SIZE;
        current_block += blocks_in_batch;
    }

    printf("[SD Store] OK (blocks %lu..%lu)\n", (unsigned long)base, (unsigned long)(current_block - 1));
    return 0;
}

/* ================================================================
   SECTION 14: INSECT DETECTION + AUTO-CAPTURE TASK (runs forever)
   ================================================================ */

/**
 * Helper: capture one frame and store to SD with sequential block numbering.
 * Uses the global snap_count so each image gets a unique location.
 */
static int capture_and_store_frame(void)
{
    uint32_t frame_size   = (uint32_t)SNAP_WIDTH * SNAP_HEIGHT * 2UL;
    uint32_t total_blocks = (SD_IMG_HEADER_SIZE + frame_size + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;

    /* Compute unique block for this capture using current snap_count */
    uint32_t target_block = snap_base_block + (snap_count * total_blocks);
    SD_IMG_BASE_BLOCK = target_block;

    uint32_t t_capture_start = HAL_GetTick();

    int rc = CAM_CaptureSingleFrame(capture_buf, MAX_SNAP_FRAME_SIZE,
                                    SNAP_WIDTH, SNAP_HEIGHT,
                                    SNAP_FPS, SNAP_WARMUP_FRAMES);

    uint32_t t_capture_end   = HAL_GetTick();
    uint32_t capture_elapsed = t_capture_end - t_capture_start;

    if (rc != 0) {
        printf("[CAP] Camera capture FAILED (rc=%d)!\n", rc);
        return -1;
    }

    printf("[CAP] Captured in %lu ms\n", (unsigned long)capture_elapsed);

    uint32_t t_sd_start = HAL_GetTick();

    if (SD_StoreRawImage(capture_buf, frame_size, SNAP_WIDTH, SNAP_HEIGHT, 0) == 0) {
        snap_count++;  /* Increment global counter AFTER successful write */
        uint32_t t_sd_end = HAL_GetTick();
        printf("[CAP] Snapshot #%lu SAVED at block %lu\n",
               (unsigned long)snap_count, (unsigned long)target_block);
        printf("[PERF] capture=%lums + sd=%lums = total=%lums\n",
               (unsigned long)capture_elapsed,
               (unsigned long)(t_sd_end - t_sd_start),
               (unsigned long)(t_sd_end - t_capture_start));
        return 0;
    } else {
        printf("[CAP] SD write FAILED at block %lu!\n", (unsigned long)target_block);
        return -1;
    }
}

static void insect_detection_task(void *arg)
{
    (void)arg;

    uint8_t status_local;
    uint8_t isAlive;
    uint8_t is_ready;
    uint8_t i;

    /* ---- Wait for system to be ready ---- */
    while (!system_ready) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    printf("\n============================================\n");
    printf("  INSECT DETECTION MODE (VL53L5CX)\n");
    printf("  4x4 zone-based signal drop detection\n");
    printf("  Auto-capture on detection\n");
    printf("============================================\n\n");

    /* ---- Properly initialize the sensor FIRST ---- */
    printf("[ToF] Initializing VL53L5CX...\n");

    VL53L5CX_GPIO_Init();
    VL53L5CX_StartSequence();

    Dev.platform.address = 0x29;

    status_local = vl53l5cx_is_alive(&Dev, &isAlive);
    if (!isAlive || status_local) {
        printf("[ERROR] VL53L5CX sensor not detected!\n");
        printf("  Status: %d, Alive: %d\n", status_local, isAlive);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    printf("[ToF] Sensor detected!\n");

    status_local = vl53l5cx_init(&Dev);
    if (status_local) {
        printf("[ERROR] VL53L5CX init failed: %d\n", status_local);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    printf("[ToF] Sensor initialized\n");

    /* ---- Configuration ---- */
    status_local = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);

    memset(g_baseline_signal, 0, sizeof(g_baseline_signal));
    memset(g_baseline_distance, 0, sizeof(g_baseline_distance));
    memset(g_zone_valid, 0, sizeof(g_zone_valid));
    g_baseline_ready = 0;

    status_local = vl53l5cx_set_integration_time_ms(&Dev, 800);
    status_local = vl53l5cx_set_ranging_frequency_hz(&Dev, 15);
    status_local = vl53l5cx_set_target_order(&Dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    status_local = vl53l5cx_set_sharpener_percent(&Dev, 10);
    status_local = vl53l5cx_set_ranging_mode(&Dev, VL53L5CX_RANGING_MODE_CONTINUOUS);

    vl53l5cx_start_ranging(&Dev);
    VL53L5CX_WaitMs(&(Dev.platform), 200);
    printf("[ToF] Ranging started...\n");

    /* ================================================================ */
    /*  Phase 1: Learn per-zone baseline signal                         */
    /* ================================================================ */
    printf("\r\n[BASELINE] Taking %d samples per zone...\r\n", BASELINE5CX_SAMPLES);

    for (i = 0; i < BASELINE5CX_SAMPLES; i++)
    {
        do {
            vl53l5cx_check_data_ready(&Dev, &is_ready);
            if (!is_ready) vTaskDelay(pdMS_TO_TICKS(10));
        } while (!is_ready);

        if (vl53l5cx_get_ranging_data(&Dev, &Results) != 0)
            continue;

        for (int z = 0; z < NUM_ZONES; z++)
        {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (Results.target_status[idx] == 0 ||
                Results.target_status[idx] == 5 ||
                Results.target_status[idx] == 9)
            {
                g_baseline_signal[z] += Results.signal_per_spad[idx];
                g_baseline_distance[z] += Results.distance_mm[idx];
                g_zone_valid[z] = 1;
            }
        }

        printf("  [%d/%d]\r", i + 1, BASELINE5CX_SAMPLES);
    }

    printf("\r\n[BASELINE] Zone | Distance(mm) | Signal(kcps/spad)\r\n");
    printf("----------|----------------|----------------------\r\n");

    uint8_t valid_zone_count = 0;

    for (int z = 0; z < NUM_ZONES; z++)
    {
        if (g_zone_valid[z])
        {
            g_baseline_signal[z] /= BASELINE5CX_SAMPLES;
            g_baseline_distance[z] /= BASELINE5CX_SAMPLES;
            valid_zone_count++;
            printf("  Z%2d    |    %8d     |    %10d\r\n",
                   z, g_baseline_distance[z], g_baseline_signal[z]);
        }
    }

    printf("\r\n[BASELINE] Valid zones: %d/%d\r\n", valid_zone_count, NUM_ZONES);
    printf("[BASELINE] Insect threshold: >%d%% drop in any zone\r\n",
           INSECT5CX_THRESHOLD_PCT);

    if (valid_zone_count < 2) {
        printf("[ERROR] Not enough valid zones for baseline (%d/%d)\n", valid_zone_count, NUM_ZONES);
        printf("  Please check sensor connection and ensure objects are in range.\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    g_baseline_ready = 1;

    /* ================================================================ */
    /*  Phase 2: Monitor + Auto-Capture                                  */
    /* ================================================================ */
    printf("[MONITOR] %d zones active. Waiting for insect...\n", valid_zone_count);

    uint32_t cooldown_frames = 0;
    volatile uint8_t is_capturing = 0;

    BSP_LED_On(LED_GREEN);

    while (1)
    {
        if (is_capturing) {
            do {
                vl53l5cx_check_data_ready(&Dev, &is_ready);
                if (!is_ready) vTaskDelay(pdMS_TO_TICKS(10));
            } while (!is_ready);
            vl53l5cx_get_ranging_data(&Dev, &Results);
            continue;
        }

        do {
            vl53l5cx_check_data_ready(&Dev, &is_ready);
            if (!is_ready) vTaskDelay(pdMS_TO_TICKS(10));
        } while (!is_ready);

        if (vl53l5cx_get_ranging_data(&Dev, &Results) != 0)
            continue;

        uint8_t insect_found = 0;

        for (int z = 0; z < NUM_ZONES; z++)
        {
            if (!g_zone_valid[z]) continue;
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (Results.target_status[idx] != 0 &&
                Results.target_status[idx] != 5 &&
                Results.target_status[idx] != 9) continue;

            uint32_t signal_drop = 0;
            if (g_baseline_signal[z] > 0)
            {
                int32_t diff = (int32_t)g_baseline_signal[z] -
                               (int32_t)Results.signal_per_spad[idx];
                if (diff < 0) diff = -diff;
                signal_drop = (uint32_t)diff * 100 / g_baseline_signal[z];
            }

            if (signal_drop > INSECT5CX_THRESHOLD_PCT) {
                insect_found = 1;
                break;
            }
        }

        if (cooldown_frames > 0) cooldown_frames--;

        if (insect_found && cooldown_frames == 0)
        {
            printf("[!] INSECT -> capture #%lu\n", (unsigned long)(snap_count + 1));

            BSP_LED_Off(LED_GREEN);
            BSP_LED_On(LED_RED);
            WS2812_TurnOn();
            vTaskDelay(pdMS_TO_TICKS(100));
            WS2812_TurnOff();

            is_capturing = 1;
            capture_and_store_frame();
            is_capturing = 0;

            BSP_LED_Off(LED_RED);
            BSP_LED_On(LED_GREEN);

            cooldown_frames = CAPTURE_COOLDOWN_FRAMES;
        }
    }
}

/* ================================================================
   SECTION 15: FREERTOS INITIALIZATION
   ================================================================ */

static int main_freertos(void)
{
    TaskHandle_t hdl;

    /* main_thread: board init (priority 1) */
    hdl = xTaskCreateStatic(main_thread_fct, "main",
                            configMINIMAL_STACK_SIZE, NULL,
                            tskIDLE_PRIORITY + 1,
                            main_thread_stack, &main_thread_cb);
    assert(hdl != NULL);

    /* insect_detection_task: ToF monitoring + auto-capture (priority 2) */
    hdl = xTaskCreateStatic(insect_detection_task, "insect",
                            8 * configMINIMAL_STACK_SIZE, NULL,
                            tskIDLE_PRIORITY + 2,
                            insect_thread_stack, &insect_thread_cb);
    assert(hdl != NULL);

    vTaskStartScheduler();
    assert(0);
    return -1;
}

/* ================================================================
   SECTION 16: MAIN THREAD (runs once: board init, then deletes)
   ================================================================ */

static void main_thread_fct(void *arg)
{
    (void)arg;
    uint32_t preemptPriority, subPriority;
    IRQn_Type i;
    int ret;

    /* ---- NVIC Priority Configuration ---- */
    HAL_NVIC_GetPriority(SysTick_IRQn, HAL_NVIC_GetPriorityGrouping(),
                         &preemptPriority, &subPriority);
    for (i = PVD_PVM_IRQn; i <= LTDC_UP_ERR_IRQn; i++)
        HAL_NVIC_SetPriority(i, preemptPriority, subPriority);

    /* ---- Clocks + Console + Fuses ---- */
    SystemClock_Config();
    vPortSetupTimerInterrupt();
    CONSOLE_Config();
    Fuse_Programming();

    HAL_Delay(10);
    printf("[PWR] All VDDIO domains enabled for Arduino I/O\n");

    /* === FORCE CONTROL PINS EARLY (ToF power) === */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);  // PWR_EN

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);  // LPn

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);  // RST

    HAL_Delay(100);
    printf("[EARLY GPIO] Control pins forced HIGH before I2C init\n");

    /* ---- Illumination System (WS2812) ---- */
    printf("[INIT] Light system\n");
    MX_GPDMA1_Init();
    MX_TIM1_Init();
    WS2812_Init();

    /* ---- VL53L5CX ToF Sensor (I2C1 + GPIO) ---- */
    VL53L5CX_I2C_Init();
    VL53L5CX_GPIO_Init();
    VL53L5CX_StartSequence();

    /* ---- PSRAM Initialization ---- */
#ifdef STM32N6570_DK_REV
    BSP_XSPI_RAM_Init(0);
    BSP_XSPI_RAM_EnableMemoryMappedMode(0);
#endif

    BSP_XSPI_NOR_Init_t NOR_Init;
    NOR_Init.InterfaceMode  = BSP_XSPI_NOR_OPI_MODE;
    NOR_Init.TransferRate   = BSP_XSPI_NOR_DTR_TRANSFER;
    BSP_XSPI_NOR_Init(0, &NOR_Init);
    BSP_XSPI_NOR_EnableMemoryMappedMode(0);

    /* ---- LEDs + User Button ---- */
    ret = BSP_LED_Init(LED_GREEN); assert(ret == BSP_ERROR_NONE);
    ret = BSP_LED_Init(LED_RED);   assert(ret == BSP_ERROR_NONE);

    BSP_PB_Init(BUTTON_USER1, BUTTON_MODE_GPIO);
    printf("[BTN] PC13 initialized via BSP (active HIGH). State=%s\n",
           BSP_PB_GetState(BUTTON_USER1) == GPIO_PIN_SET ? "PRESSED" : "RELEASED");

    /* ---- Security + IAC ---- */
    Security_Config();
    IAC_Config();

    /* ---- Low-Power Clock Gates ---- */
    LL_BUS_EnableClockLowPower(~0);
    LL_MEM_EnableClockLowPower(~0);
    LL_AHB1_GRP1_EnableClockLowPower(~0);
    LL_AHB2_GRP1_EnableClockLowPower(~0);
    LL_AHB3_GRP1_EnableClockLowPower(~0);
    LL_AHB4_GRP1_EnableClockLowPower(~0);
    LL_AHB5_GRP1_EnableClockLowPower(~0);
    LL_APB1_GRP1_EnableClockLowPower(~0);
    LL_APB1_GRP2_EnableClockLowPower(~0);
    LL_APB2_GRP1_EnableClockLowPower(~0);
    LL_APB4_GRP1_EnableClockLowPower(~0);
    LL_APB4_GRP2_EnableClockLowPower(~0);
    LL_APB5_GRP1_EnableClockLowPower(~0);
    LL_MISC_EnableClockLowPower(~0);

    /* ---- SD Card Initialization ---- */
    printf("\n=== SD Card Detection Test ===\n");

    HAL_PWREx_EnableVddIO5();
    for (volatile uint32_t d = 0; d < 500000; d++);

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_SDMMC2;
    PeriphClkInit.Sdmmc2ClockSelection  = RCC_SDMMC2CLKSOURCE_HCLK;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_SDMMC2_FORCE_RESET();
    for (volatile uint32_t d = 0; d < 1000; d++);
    __HAL_RCC_SDMMC2_RELEASE_RESET();
    for (volatile uint32_t d = 0; d < 1000; d++);

    hsd1.Instance             = SDMMC2;
    hsd1.Init.ClockEdge       = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave  = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide         = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv        = 2;
    hsd1.State                = HAL_SD_STATE_RESET;

    HAL_SD_MspInit(&hsd1);
    HAL_StatusTypeDef status = HAL_SD_Init(&hsd1);

    if (status == HAL_OK) {
        printf("[SD] Switching to 4-bit bus width...\n");
        status = HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);
        if (status == HAL_OK) {
            hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
            printf("[SD] 4-bit bus width configured successfully\n");
        } else {
            printf("[SD] Wide bus config failed (%lu), staying in 1-bit\n", (unsigned long)status);
            status = HAL_OK;
        }
    }

    if (status == HAL_OK) {
        uint64_t     total_bytes = (uint64_t)hsd1.SdCard.BlockNbr * (uint64_t)hsd1.SdCard.BlockSize;
        unsigned long size_mb    = (unsigned long)(total_bytes / 1048576UL);
        unsigned long size_gb    = (unsigned long)(total_bytes / 1073741824UL);

        printf("\n*** SD CARD DETECTED SUCCESSFULLY! ***\n\n");
        printf("  Card Size: %lu GB (%lu MB)\n", size_gb, size_mb);
        printf("  Blocks:    %lu x %lu bytes\n",
               (unsigned long)hsd1.SdCard.BlockNbr, (unsigned long)hsd1.SdCard.BlockSize);
        printf("  Card Type: %s\n",
               (hsd1.SdCard.CardType == CARD_SDHC_SDXC) ? "SDHC/SDXC" : "SDSC");
        printf("  Bus Width: %s\n",
               (hsd1.Init.BusWide == SDMMC_BUS_WIDE_4B) ? "4-bit" : "1-bit");

        uint32_t blocks_per_snap = (SD_IMG_HEADER_SIZE + SNAP_FRAME_SIZE + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
        uint32_t available_blocks = hsd1.SdCard.BlockNbr - SD_SNAP_BASE_BLOCK;
        uint32_t max_snaps = available_blocks / blocks_per_snap;
        printf("  Blocks/snapshot: %lu\n", (unsigned long)blocks_per_snap);
        printf("  Max snapshots: ~%lu (from block %lu onward)\n",
               (unsigned long)max_snaps, (unsigned long)SD_SNAP_BASE_BLOCK);
        printf("\n");

        SD_Benchmarks();
    } else {
        printf("[ERROR] SD card init failed: %lu\n", (unsigned long)status);
    }

    /* ---- System Ready ---- */
    printf("\n===========================================\n");
    printf("[INFO] System READY!\n");
    printf("[INFO] Insect detection + auto-capture enabled.\n");
    printf("[INFO] GREEN LED = monitoring, RED LED = capturing.\n");
    printf("===========================================\n\n");

    system_ready = 1;
    vTaskDelete(NULL);
}

/* ================================================================
   SECTION 17: DCMIPP CLOCK CONFIGURATION
   ================================================================ */

HAL_StatusTypeDef MX_DCMIPP_ClockConfig(DCMIPP_HandleTypeDef *hdcmipp)
{
    (void)hdcmipp;
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};
    HAL_StatusTypeDef        ret;

    RCC_PeriphCLKInitStruct.PeriphClockSelection   = RCC_PERIPHCLK_DCMIPP;
    RCC_PeriphCLKInitStruct.DcmippClockSelection   = RCC_DCMIPPCLKSOURCE_IC17;
    RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockSelection = RCC_ICCLKSOURCE_PLL2;
    RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockDivider   = 3;
    ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
    if (ret) return ret;

    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CSI;
    RCC_PeriphCLKInitStruct.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL1;
    RCC_PeriphCLKInitStruct.ICSelection[RCC_IC18].ClockDivider   = 40;
    ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
    if (ret) return ret;

    return HAL_OK;
}

/* ================================================================
   SECTION 18: USB MSP INIT
   ================================================================ */

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    assert(hpcd->Instance == USB1_OTG_HS);
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableVddUSBVMEN();
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_USB33RDY) == 0U);
    HAL_PWREx_EnableVddUSB();
    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
    USB1_HS_PHYC->USBPHYC_CR &= ~(0x7U << 0x4U);
    USB1_HS_PHYC->USBPHYC_CR |= (0x2U << 0x4U);
    __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();
    HAL_NVIC_SetPriority(USB1_OTG_HS_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
}

/* ================================================================
   SECTION 19: PERIPHERAL INITIALIZATION (TIM1, GPDMA1, CLOCK)
   ================================================================ */

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 5;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 500;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim1);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htim1);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

    HAL_TIM_MspPostInit(&htim1);
}

void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
    PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_DIV1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        //Error_Handler();
    }
}

static void MX_GPDMA1_Init(void)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
}

/* ================================================================
   SECTION 20: I2C1 INITIALIZATION (VL53L5CX ToF Sensor)
   ================================================================ */

static void VL53L5CX_I2C_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00401242;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        //Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        //Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        //Error_Handler();
    }

    if (HAL_I2CEx_ConfigFastModePlus(&hi2c1, I2C_FASTMODEPLUS_ENABLE) != HAL_OK) {
        //Error_Handler();
    }

    printf("Testing I2C bus...\n");

    status = HAL_I2C_IsDeviceReady(&hi2c1, (0x29 << 1), 10, 100);
    printf("VL53L5CX at 0x29 ready? %s (status=%d)\n",
           (status == HAL_OK) ? "YES" : "NO", status);

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 3, 50) == HAL_OK) {
            printf("Device found at 0x%02X\n", addr);
        }
    }
}

/* ================================================================
   SECTION 21: VL53L5CX TIME-OF-FLIGHT SENSOR FUNCTIONS
   ================================================================ */

static void VL53L5CX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

   __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);

    printf("GPIO control pins forced to 3.3V\n");
}

static void VL53L5CX_StartSequence(void)
{
    printf("\n=== ToF Sensor Power Up ===\n");

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);  // PWR_EN
    HAL_Delay(20);

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(20);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);  // LPn
    HAL_Delay(100);

    printf("[OK] Sensor power-up complete\n");
}

static void VL53L5CX_Validate(void)
{
    Dev.platform.address = 0x29;

    status = vl53l5cx_is_alive(&Dev, &isAlive);
    printf("ALIVE %i and status %i\r\n", isAlive, status);

    if (!isAlive || status) {
        printf("Sensor NOT detected\r\n");
    } else {
        printf("Sensor detected\r\n");

        status = vl53l5cx_init(&Dev);
        if (status) {
            printf("Init FAIL\r\n");
        } else {
            printf("Init OK\r\n");
            vl53l5cx_start_ranging(&Dev);
        }
    }
    HAL_Delay(1000);

    for (uint8_t addr = 0; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
            printf("I2C device found at 0x%02X\r\n", addr);
        }
    }
}

static void VL53L5CX_ReadingTest(void)
{
    (void)status;
    uint8_t is_ready;
    uint8_t i;

    status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
    memset(g_baseline_signal, 0, sizeof(g_baseline_signal));
    memset(g_baseline_distance, 0, sizeof(g_baseline_distance));
    memset(g_zone_valid, 0, sizeof(g_zone_valid));
    g_baseline_ready = 0;

    status = vl53l5cx_set_integration_time_ms(&Dev, 800);
    status = vl53l5cx_set_ranging_frequency_hz(&Dev, 15);
    status = vl53l5cx_set_target_order(&Dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    status = vl53l5cx_set_sharpener_percent(&Dev, 10);
    status = vl53l5cx_set_ranging_mode(&Dev, VL53L5CX_RANGING_MODE_CONTINUOUS);

    vl53l5cx_start_ranging(&Dev);
    VL53L5CX_WaitMs(&(Dev.platform), 200);
    printf("Ranging started...\r\n");

    printf("\r\n[BASELINE] Taking %d samples per zone...\r\n", BASELINE5CX_SAMPLES);

    for (i = 0; i < BASELINE5CX_SAMPLES; i++)
    {
        do {
            vl53l5cx_check_data_ready(&Dev, &is_ready);
            if (!is_ready) HAL_Delay(10);
        } while (!is_ready);

        if (vl53l5cx_get_ranging_data(&Dev, &Results) != 0)
            continue;

        for (int z = 0; z < NUM_ZONES; z++)
        {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (Results.target_status[idx] == 0 ||
                Results.target_status[idx] == 5 ||
                Results.target_status[idx] == 9)
            {
                g_baseline_signal[z] += Results.signal_per_spad[idx];
                g_baseline_distance[z] += Results.distance_mm[idx];
                g_zone_valid[z] = 1;
            }
        }

        printf("  [%d/%d]\r", i + 1, BASELINE5CX_SAMPLES);
    }

    printf("\r\n[BASELINE] Zone | Distance(mm) | Signal(kcps/spad)\r\n");
    printf("----------|----------------|----------------------\r\n");

    uint8_t valid_zone_count = 0;

    for (int z = 0; z < NUM_ZONES; z++)
    {
        if (g_zone_valid[z])
        {
            g_baseline_signal[z] /= BASELINE5CX_SAMPLES;
            g_baseline_distance[z] /= BASELINE5CX_SAMPLES;
            valid_zone_count++;
            printf("  Z%2d    |    %8d     |    %10d\r\n",
                   z, g_baseline_distance[z], g_baseline_signal[z]);
        }
    }

    printf("\r\n[BASELINE] Valid zones: %d/%d\r\n", valid_zone_count, NUM_ZONES);
    printf("[BASELINE] Insect threshold: >%d%% drop in any zone\r\n",
           INSECT5CX_THRESHOLD_PCT);
    g_baseline_ready = 1;

    printf("\r\n[MONITORING] Watching %d zones...\r\n", valid_zone_count);
    printf("  (Wave hand or blow air to simulate)\r\n\n");

    while (1)
    {
        do {
            vl53l5cx_check_data_ready(&Dev, &is_ready);
            if (!is_ready) HAL_Delay(10);
        } while (!is_ready);

        if (vl53l5cx_get_ranging_data(&Dev, &Results) != 0)
            continue;

        uint8_t  insect_found = 0;
        uint8_t  affected_zones[NUM_ZONES] = {0};
        uint32_t affected_drop[NUM_ZONES] = {0};
        uint8_t  affected_count = 0;

        for (int z = 0; z < NUM_ZONES; z++)
        {
            if (!g_zone_valid[z])
                continue;

            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

            if (Results.target_status[idx] != 0 &&
                Results.target_status[idx] != 5 &&
                Results.target_status[idx] != 9)
                continue;

            uint32_t signal_drop = 0;
            if (g_baseline_signal[z] > 0)
            {
                int32_t diff = (int32_t)g_baseline_signal[z] -
                               (int32_t)Results.signal_per_spad[idx];
                if (diff < 0) diff = -diff;
                signal_drop = (uint32_t)diff * 100 / g_baseline_signal[z];
            }

            if (signal_drop > INSECT5CX_THRESHOLD_PCT)
            {
                affected_zones[affected_count] = z;
                affected_drop[affected_count] = signal_drop;
                affected_count++;
                insect_found = 1;
            }
        }

        if (insect_found)
        {
            char zone_list[64] = {0};
            for (uint8_t k = 0; k < affected_count; k++)
            {
                char tmp[16];
                sprintf(tmp, "Z%u", affected_zones[k]);
                if (k == 0)
                    strcpy(zone_list, tmp);
                else {
                    strcat(zone_list, ",");
                    strcat(zone_list, tmp);
                }
            }

            printf(">>> INSECT DETECTED! Zones: [%s] (%u zone(s))\r\n",
                   zone_list, affected_count);

            for (uint8_t k = 0; k < affected_count; k++)
            {
                uint8_t z = affected_zones[k];
                uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
                printf("    Z%2d: dist=%4dmm  signal=%6d  baseline=%6d  (drop=%u%%)\r\n",
                       z, Results.distance_mm[idx], Results.signal_per_spad[idx],
                       g_baseline_signal[z], affected_drop[k]);
            }
            printf("\r\n");
        }
        else
        {
            uint16_t min_d = 9999, max_d = 0;
            uint32_t min_s = 999999, max_s = 0;
            uint8_t  frame_valid = 0;

            for (int z = 0; z < NUM_ZONES; z++)
            {
                if (!g_zone_valid[z]) continue;
                uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
                if (Results.target_status[idx] != 0 &&
                    Results.target_status[idx] != 5 &&
                    Results.target_status[idx] != 9) continue;

                frame_valid++;
                if (Results.distance_mm[idx] < min_d) min_d = Results.distance_mm[idx];
                if (Results.distance_mm[idx] > max_d) max_d = Results.distance_mm[idx];
                if (Results.signal_per_spad[idx] < min_s) min_s = Results.signal_per_spad[idx];
                if (Results.signal_per_spad[idx] > max_s) max_s = Results.signal_per_spad[idx];
            }

            if (frame_valid > 0)
            {
                printf(". zones=%2d  dist=%4d-%4dmm  signal=%5d-%5d\r\n",
                       frame_valid, min_d, max_d, min_s, max_s);
            }
        }
    }
}

static void VL53L5CX_MotionTest(void)
{
    (void)MOTION_INDICATOR_THRESH;
    // Reserved for future hardware motion indicator integration
}

/* ================================================================
   SECTION 22: ASSERT CALLBACK
   ================================================================ */

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    UNUSED(file);
    UNUSED(line);
    __BKPT(0);
    while (1) {}
}
#endif

/* ================================================================
   SECTION 23: DEBUG HELPER
   ================================================================ */

__attribute__((section(".keep_me")))
void app_clean_invalidate_dbg(void)
{
    SCB_CleanInvalidateDCache();
}

/* ================================================================
   SECTION 24: UNUSED FORWARD DECLARATION SUPPRESSION
   ================================================================ */

/* SD_ReadRawImage is declared but not used — suppress warning */
static int SD_ReadRawImage(uint8_t *img_buf, uint32_t *data_size_p)
{
    (void)img_buf; (void)data_size_p;
    return -1;
}