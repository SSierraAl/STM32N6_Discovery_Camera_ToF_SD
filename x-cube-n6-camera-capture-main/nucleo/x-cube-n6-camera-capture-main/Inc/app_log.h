#ifndef APP_LOG_H
#define APP_LOG_H

#include "app_config.h"

/* String-literal decorators keep each record in its original printf call.
   The background is reset immediately after the tag. No buffers, task state,
   heap allocation or additional UART writes are introduced. */
#if APP_LOG_ANSI_COLORS
/** Camera tag, black text on cyan. Argument must be a string literal. */
#define LOG_CAMERA(tag) "\x1b[30;46m" tag "\x1b[0m"
/** ToF and baseline tags, black text on yellow. */
#define LOG_TOF(tag) "\x1b[30;43m" tag "\x1b[0m"
/** Storage and journal tags, white text on blue. */
#define LOG_STORAGE(tag) "\x1b[97;44m" tag "\x1b[0m"
#else
#define LOG_CAMERA(tag) tag
#define LOG_TOF(tag) tag
#define LOG_STORAGE(tag) tag
#endif

#endif /* APP_LOG_H */
