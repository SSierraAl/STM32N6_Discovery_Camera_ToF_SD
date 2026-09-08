#ifndef RTC_TEST_SEMPHR_H
#define RTC_TEST_SEMPHR_H
typedef void *SemaphoreHandle_t;
int xSemaphoreTake(SemaphoreHandle_t,unsigned);
int xSemaphoreGive(SemaphoreHandle_t);
#endif
