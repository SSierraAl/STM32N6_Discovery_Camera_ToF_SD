/**
 * ******************************************************************************
 * @file    debug_color.h
 * @brief   ANSI color codes for colored debug output (Tera Term compatible)
 *
 *   Usage examples:
 *     printf(BRED "ERROR: " RESET "SD card not found\n");
 *     printf(BGREEN "[OK] " GREEN "Camera initialized\n" RESET);
 *     printf(BYELLOW ">>> " YELLOW "INSECT DETECTED!\n" RESET);
 ******************************************************************************
 */
#ifndef DEBUG_COLOR_H
#define DEBUG_COLOR_H

/* ---- Basic Colors (foreground) ---- */
#define RESET       "\033[0m"
#define BLACK       "\033[30m"
#define RED         "\033[31m"
#define GREEN       "\033[32m"
#define YELLOW      "\033[33m"
#define BLUE        "\033[34m"
#define MAGENTA     "\033[35m"
#define CYAN        "\033[36m"
#define WHITE       "\033[37m"

/* ---- Bold Colors (bright foreground) ---- */
#define BBLACK      "\033[1;30m"
#define BRED        "\033[1;31m"
#define BGREEN      "\033[1;32m"
#define BYELLOW     "\033[1;33m"
#define BBLUE       "\033[1;34m"
#define BMAGENTA    "\033[1;35m"
#define BCYAN       "\033[1;36m"
#define BWHITE      "\033[1;37m"

/* ---- Sensor/Module Specific Macros ---- */

/* Camera: Cyan for normal, Bold Cyan for errors */
#define CAM_TAG     BCYAN "[CAM] " RESET
#define CAM_TAG_ERR BRED "[CAM!]" RESET
#define CAM_TAG_OK  BGREEN "[CAM]"  RESET

/* ToF Sensor: Magenta for normal, Bold Magenta for detections */
#define TOF_TAG     BMAGENTA "[ToF] " RESET
#define TOF_DET     BYELLOW ">>> " YELLOW
#define TOF_TAG_ERR BRED  "[ToF!]" RESET

/* SD Card: Green for OK, Red for errors */
#define SD_TAG      BGREEN "[SD] " RESET
#define SD_TAG_ERR  BRED   "[SD!]" RESET
#define SD_TAG_WRN  BYELLOW "[SD~]" RESET

/* Sensor Task (ToF monitoring) */
#define SENSOR_TAG  BMAGENTA "[SENSOR]" RESET
#define SENSOR_ERR  BRED    "[SENSOR!]" RESET

/* Storage Task */
#define STORAGE_TAG BGREEN "[STORAGE]" RESET

/* IPC */
#define IPC_TAG     BCYAN "[IPC] " RESET

/* System/Init */
#define INIT_TAG    BBLUE "[INIT] " RESET
#define INFO_TAG    BLUE  "[INFO] " RESET
#define WARN_TAG    YELLOW "[WARN] " RESET
#define ERR_TAG     BRED   "[ERROR] " RESET
#define OK_TAG      BGREEN "[OK] " RESET

/* LED/Illumination */
#define LED_TAG     BYELLOW "[LED] " RESET

/* Performance */
#define PERF_TAG    BWHITE "[PERF] " RESET

#endif /* DEBUG_COLOR_H */