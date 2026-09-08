#ifndef DS3231_H
#define DS3231_H
#include <stdint.h>
#include <stddef.h>
typedef struct { unsigned year, month, day, hour, minute, second; } RTC_DateTime;
typedef struct { uint32_t unix_seconds, capture_tick, valid; } RTC_Stamp;
int RTC_Encode(const RTC_DateTime *dt, uint8_t registers[7]);
int RTC_Decode(const uint8_t registers[7], RTC_DateTime *dt);
uint32_t RTC_ToUnix(const RTC_DateTime *dt);
int RTC_ParseUTC(const char *text, RTC_DateTime *dt);
void RTC_FormatUTC(const RTC_DateTime *dt, char *text, size_t size);
/* Task context only, after I2C1 and its shared arbiter are initialized. */
/* Get/Set: 0 success, -1 I2C transfer, -2 invalid RTC calendar/status,
   -3 mutex unavailable, -4 invalid argument. */
int RTC_Get(RTC_DateTime *dt);
int RTC_Set(const RTC_DateTime *dt);
void RTC_Init(void);
RTC_Stamp RTC_CaptureStamp(void); /* Post-capture/batch completion, not exposure time. */
#endif
