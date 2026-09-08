#include "ds3231.h"
#include "raw_image.h"
#include "stm32n6xx_hal.h"
#include "i2c_arbiter.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
I2C_HandleTypeDef hi2c1;
SemaphoreHandle_t g_i2c1_mutex=(void*)1;
static uint8_t regs[16];
static int locked, busy, fail, reads, writes;
uint32_t HAL_GetTick(void) { return 12345; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *h) { assert(h==&hi2c1); return fail?HAL_I2C_ERROR_AF:0; }
int xSemaphoreTake(SemaphoreHandle_t m,unsigned t) {
    assert(m && t==20 && !locked); if (busy) return 0; locked=1; return 1;
}
int xSemaphoreGive(SemaphoreHandle_t m) { assert(m && locked); locked=0; return 1; }
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h,unsigned a,unsigned r,unsigned sz,uint8_t *d,unsigned n,unsigned t) {
    assert(h==&hi2c1 && a==0xd0 && sz==1 && t==20 && locked && r+n<=16);
    ++reads; if (fail) return -1; memcpy(d,regs+r,n); return 0;
}
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h,unsigned a,unsigned r,unsigned sz,uint8_t *d,unsigned n,unsigned t) {
    assert(h==&hi2c1 && a==0xd0 && sz==1 && t==20 && locked && r+n<=16);
    ++writes; if (fail) return -1;
    if (r==15 && n==1) { regs[15]=(d[0]&0xfc)|(regs[15]&d[0]&3); }
    else memcpy(regs+r,d,n);
    return 0;
}
int main(void) {
    RTC_DateTime d={2000,1,1,0,0,0}, got;
    assert(RTC_ToUnix(&d)==946684800);
    assert(!RTC_ParseUTC("2024-02-29T12:34:56Z",&d));
    assert(RTC_ToUnix(&d)==1709210096);
    assert(!RTC_Encode(&d,regs) && !RTC_Decode(regs,&got));
    assert(RTC_ToUnix(&got)==RTC_ToUnix(&d));
    assert(RTC_ParseUTC("2023-02-29T00:00:00Z",&got));
    assert(RTC_ParseUTC("2024-02-29T12:34:56",&got));
    assert(RTC_ParseUTC("2100-01-01T00:00:00Z",&got));
    char text[21]; RTC_FormatUTC(&d,text,sizeof(text));
    assert(!strcmp(text,"2024-02-29T12:34:56Z"));
    regs[2]=0x52; assert(!RTC_Decode(regs,&got) && got.hour==0);
    regs[2]=0x72; assert(!RTC_Decode(regs,&got) && got.hour==12);
    regs[2]=0x61; assert(!RTC_Decode(regs,&got) && got.hour==13);
    regs[2]=0x40; assert(RTC_Decode(regs,&got));
    RTC_Encode(&d,regs); regs[5]|=0x80; assert(RTC_Decode(regs,&got));
    RTC_Encode(&d,regs); regs[0]=0x6a; assert(RTC_Decode(regs,&got));
    RTC_Encode(&d,regs);
    assert(!RTC_Get(&got) && !writes && !locked);
    regs[15]=0x83; assert(RTC_Get(&got)==-2 && !writes);
    regs[14]=0x9c;
    assert(!RTC_Set(&d));
    assert(regs[14]==0x1c && regs[15]==3 && !locked);
    RTC_Stamp s=RTC_CaptureStamp();
    assert(s.valid && s.unix_seconds==1709210096 && s.capture_tick==12345);
    sd_image_header_t hdr={0}; RawImage_SetStamp(&hdr,s);
    assert(sizeof(hdr)==64 && hdr.time_tag==RAW_TIME_TAG && hdr.time_flags==3);
    assert((uint8_t*)&hdr.time_tag-(uint8_t*)&hdr==32);
    busy=1; int oldreads=reads; assert(RTC_Get(&got)==-3 && reads==oldreads);
    s=RTC_CaptureStamp(); assert(!s.valid && !s.unix_seconds); busy=0;
    fail=1; assert(RTC_Get(&got)==-1 && !locked);
    int oldwrites=writes; assert(RTC_Set(&d)==-1 && writes==oldwrites && !locked);
    RTC_Init(); /* Print transport failure, never suggest setting as its fix. */
    fail=0; regs[15]=0x80; RTC_Init(); /* Separately report oscillator-stop flag. */
    assert(RTC_Get(NULL)==-4 && RTC_Set(NULL)==-4);
    puts("PASS: RTC calendar, UTC, coherent read, OSF, shared mutex, failure paths, raw header");
    return 0;
}
