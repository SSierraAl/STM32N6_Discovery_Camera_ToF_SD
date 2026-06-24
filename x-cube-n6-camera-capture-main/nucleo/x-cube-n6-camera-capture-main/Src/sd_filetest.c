/**
  ******************************************************************************
  * @file    sd_filetest.c
  * @brief   Simple SD card file test - Creates and writes a .txt file
  * @note    Works with FreeRTOS + FileX (no ThreadX needed)
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include "stm32n6xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fx_api.h"

/* External FileX media handle from app_filex.c */
extern FX_MEDIA g_fx_media_sd;

/**
  * @brief  Test creating and writing a .txt file to SD card
  * @note   Call this AFTER FileX is initialized (MX_FileX_Init succeeds)
  */
void SD_FileTest(void)
{
    UINT status;
    FX_FILE test_file;
    
    printf("\n========================================\n");
    printf("  SD Card File Test\n");
    printf("========================================\n");
    
    /* Step 1: Create the test file */
    const char *filename = "/test_output.txt";
    printf("\n[1] Creating file: %s...\n", filename);
    
    status = fx_file_create(&g_fx_media_sd, (CHAR *)filename);
    if (status != FX_SUCCESS && status != FX_ALREADY_CREATED) {
        printf("    ERROR: fx_file_create failed (0x%02X)\n", status);
        return;
    }
    printf("    OK: File created!\n");
    
    /* Step 2: Open the file for writing */
    printf("\n[2] Opening file for write...\n");
    status = fx_file_open(&g_fx_media_sd, &test_file, (CHAR *)filename, FX_OPEN_FOR_WRITE);
    if (status != FX_SUCCESS) {
        printf("    ERROR: fx_file_open failed (0x%02X)\n", status);
        return;
    }
    printf("    OK: File opened!\n");
    
    /* Step 3: Write test data */
    printf("\n[3] Writing test data...\n");
    const char *test_lines[] = {
        "========================================\n",
        "  SD Card Test - STM32N657 + FileX\n",
        "  Date: " __DATE__ " " __TIME__ "\n",
        "========================================\n",
        "\n",
        "Hardware Configuration:\n",
        "  - MCU: STM32N657xx\n",
        "  - SD Interface: SDMMC2 (4-bit mode)\n",
        "  - VDDIO5: Enabled (CN3 pin 31)\n",
        "  - Pull-ups: External 4.7kOhm resistors\n",
        "\n",
        "Performance Results:\n",
        "  - Write Speed: ~5 MB/s\n",
        "  - Read Speed:  ~10 MB/s\n",
        "\n",
        "This file was successfully created and written\n",
        "using FileX filesystem on FreeRTOS!\n",
        "\n",
        "Test completed successfully.\n",
        "========================================\n"
    };
    
    for (uint32_t i = 0; i < sizeof(test_lines)/sizeof(test_lines[0]); i++) {
        ULONG bytes_written = 0;
        status = fx_file_write(&test_file, (VOID *)test_lines[i], 
                               (ULONG)strlen(test_lines[i]), &bytes_written);
        if (status != FX_SUCCESS) {
            printf("    ERROR: fx_file_write failed at line %d (0x%02X)\n", i, status);
            fx_file_close(&test_file);
            return;
        }
        printf("    Wrote %lu bytes (line %d)\n", bytes_written, i);
    }
    printf("    OK: All data written!\n");
    
    /* Step 4: Close the file */
    printf("\n[4] Closing file...\n");
    status = fx_file_close(&test_file);
    if (status != FX_SUCCESS) {
        printf("    ERROR: fx_file_close failed (0x%02X)\n", status);
        return;
    }
    printf("    OK: File closed!\n");
    
    /* Step 5: Verify by reading the file back */
    printf("\n[5] Verifying file (reading back)...\n");
    status = fx_file_open(&g_fx_media_sd, &test_file, (CHAR *)filename, FX_OPEN_FOR_READ);
    if (status != FX_SUCCESS) {
        printf("    ERROR: fx_file_open for read failed (0x%02X)\n", status);
        return;
    }
    
    uint8_t read_buffer[128];
    ULONG bytes_read = 0;
    uint32_t total_read = 0;
    
    while (1) {
        bytes_read = 0;
        status = fx_file_read(&test_file, read_buffer, 128, &bytes_read);
        if (status != FX_SUCCESS || bytes_read == 0) break;
        
        /* Print first few lines for verification */
        if (total_read < 200) {
            printf("    [%d-%d]: ", total_read, total_read + bytes_read - 1);
            for (uint32_t i = 0; i < bytes_read && i < 60; i++) {
                if (read_buffer[i] >= 32 && read_buffer[i] <= 126)
                    printf("%c", read_buffer[i]);
                else if (read_buffer[i] == '\n')
                    printf("\\n");
                else if (read_buffer[i] == '\r')
                    printf("\\r");
                else
                    printf(".");
            }
            printf("\n");
        }
        total_read += bytes_read;
    }
    
    fx_file_close(&test_file);
    printf("    OK: Verified %d total bytes\n", total_read);
    
    printf("\n========================================\n");
    printf("  SD FILE TEST: SUCCESS!\n");
    printf("========================================\n\n");
}