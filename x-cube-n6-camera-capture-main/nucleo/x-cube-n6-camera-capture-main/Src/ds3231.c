#include "ds3231.h"
#include "app_config.h"
#include "stm32n6xx_hal.h"
#include "i2c_arbiter.h"
#include <stdio.h>
extern I2C_HandleTypeDef hi2c1;
#define RTC_ADDRESS_7BIT 0x68U
#define RTC_ADDRESS      (RTC_ADDRESS_7BIT << 1)
/* Saved while holding the bus mutex; printing happens after releasing it. */
static unsigned last_hal;
static uint32_t last_error;
static uint8_t last_control, last_status;
static int lock(void) {
    return g_i2c1_mutex && xSemaphoreTake(g_i2c1_mutex,pdMS_TO_TICKS(20)) == pdTRUE;
}
static int read_reg(unsigned reg, uint8_t *data, unsigned n) {
    last_hal=HAL_I2C_Mem_Read(&hi2c1,RTC_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,data,n,20);
    last_error=HAL_I2C_GetError(&hi2c1);
    return last_hal==HAL_OK?0:-1;
}
static int write_reg(unsigned reg, uint8_t *data, unsigned n) {
    last_hal=HAL_I2C_Mem_Write(&hi2c1,RTC_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,data,n,20);
    last_error=HAL_I2C_GetError(&hi2c1);
    return last_hal==HAL_OK?0:-1;
}
#if RTC_ENABLE
static void rtc_probe_address(uint8_t address_7bit) {
    unsigned ack_count = 0;
    const unsigned attempts = 10;
    if (!lock()) {
        printf("[RTC DIAG] 0x%02X: mutex unavailable\n", address_7bit);
        return;
    }
    for (unsigned i = 0; i < attempts; i++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address_7bit << 1), 1, 10) == HAL_OK)
            ack_count++;
        HAL_Delay(2);
    }
    xSemaphoreGive(g_i2c1_mutex);
    printf("[RTC DIAG] 0x%02X: ACK %u/%u\n", address_7bit, ack_count, attempts);
}

static void rtc_direct_read_diagnostic(void) {
    const unsigned attempts = 10;
    unsigned ok_count = 0;

    printf("[RTC READ] Direct 0x68 register read test: reg 0x00, 7 bytes, %u attempts, READ ONLY\n",
           attempts);

    for (unsigned i = 0; i < attempts; i++) {
        uint8_t regs[7] = {0};
        HAL_StatusTypeDef hal;
        uint32_t err;

        if (!lock()) {
            printf("[RTC READ %02u] mutex unavailable\n", i + 1);
            HAL_Delay(50);
            continue;
        }

        hal = HAL_I2C_Mem_Read(&hi2c1, RTC_ADDRESS, 0x00U,
                               I2C_MEMADD_SIZE_8BIT, regs, sizeof(regs), 20);
        err = HAL_I2C_GetError(&hi2c1);
        xSemaphoreGive(g_i2c1_mutex);

        if (hal == HAL_OK) {
            ok_count++;
            printf("[RTC READ %02u] OK RAW=%02X %02X %02X %02X %02X %02X %02X\n",
                   i + 1,
                   regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6]);
        } else {
            printf("[RTC READ %02u] FAIL HAL=%u ERR=0x%08lX\n",
                   i + 1, (unsigned)hal, (unsigned long)err);
        }

        HAL_Delay(50);
    }

    printf("[RTC READ] Summary: %u/%u direct reads succeeded\n", ok_count, attempts);
}

static void rtc_targeted_diagnostics(void) {
    printf("[RTC DIAG] Safe targeted test only; no writes and no ToF reconfiguration\n");
    printf("[RTC DIAG] I2C1 state before probes: %u\n", (unsigned)HAL_I2C_GetState(&hi2c1));
    printf("[RTC DIAG] 0x57 = AT24C32 EEPROM candidate\n");
    printf("[RTC DIAG] 0x68 = DS3231 RTC address\n");
    rtc_probe_address(0x57U);
    rtc_probe_address(0x68U);
    rtc_direct_read_diagnostic();
    printf("[RTC DIAG] I2C1 state after probes: %u\n", (unsigned)HAL_I2C_GetState(&hi2c1));
}

static void print_failure(int rc) {
    if (rc==-3) {
        printf("[RTC] Shared I2C1 mutex unavailable; clock was not read\n");
    } else if (rc==-1) {
        printf("[RTC] I2C transfer failed: expected DS3231 7-bit=0x68 HAL-address=0xD0 HAL=%u error=0x%08lX\n",
               last_hal,(unsigned long)last_error);
        if (last_error & HAL_I2C_ERROR_AF)
            printf("[RTC] NACK at 0x68: keep RTC_SET_ON_BOOT=0 until 0x68 is stable\n");
    } else if (rc==-2) {
        printf("[RTC] Device responded at 0x68: control=0x%02X status=0x%02X; %s\n",
               (unsigned)last_control,(unsigned)last_status,
               (last_status&0x80)?"oscillator-stop flag set":
               (last_control&0x80)?"battery oscillator disabled":"invalid calendar");
        printf("[RTC] Set RTC_SET_UTC and provision with RTC_SET_ON_BOOT=1, then restore 0\n");
    } else printf("[RTC] Invalid date/time argument\n");
}
#endif
int RTC_Get(RTC_DateTime *dt) {
    uint8_t r[16];
    if (!dt) return -4;
    if (!lock()) return -3;
    /* One coherent burst: START latches calendar; includes control/status. */
    int rc=read_reg(0,r,sizeof(r));
    if (!rc) { last_control=r[14]; last_status=r[15]; }
    xSemaphoreGive(g_i2c1_mutex);
    if (rc) return -1;
    if ((r[15]&0x80) || (r[14]&0x80)) return -2;
    return RTC_Decode(r,dt)==0?0:-2;
}
int RTC_Set(const RTC_DateTime *dt) {
    uint8_t r[7], control=0, status=0;
    if (RTC_Encode(dt,r)) return -4;
    if (!lock()) return -3;
    int rc=read_reg(14,&control,1);
    control &= 0x7f; /* EOSC=0: retain time on battery too. */
    if (!rc) rc=write_reg(14,&control,1);
    if (!rc) rc=write_reg(0,r,7);
    if (!rc) rc=read_reg(15,&status,1);
    if (!rc) {
        /* OSF only cleared after explicit valid time write. Writing 1 to
           alarm flags leaves them unchanged, including concurrent alarms. */
        status=(status&0x7f)|3;
        rc=write_reg(15,&status,1);
    }
    xSemaphoreGive(g_i2c1_mutex);
    return rc;
}
RTC_Stamp RTC_CaptureStamp(void) {
    RTC_Stamp s={0,HAL_GetTick(),0};
#if RTC_ENABLE
    RTC_DateTime dt;
    int rc=RTC_Get(&dt);
    if (rc==0) {
        s.unix_seconds=RTC_ToUnix(&dt); s.valid=1;
#if RTC_PRINT_TIME
        char text[21]; RTC_FormatUTC(&dt,text,sizeof(text));
        printf("[RTC] Capture completed %s\n",text);
#endif
    } else {
#if RTC_PRINT_TIME
        print_failure(rc);
        printf("[RTC] Time unavailable; image marked UNTIMED\n");
#endif
    }
#endif
    return s;
}
void RTC_Init(void) {
#if RTC_ENABLE
    printf("[RTC] Expecting DS3231 at fixed 7-bit address 0x68 (HAL 0xD0); AT24C32 at 0x57 is not the clock\n");
    /* Short quiet interval only. I2C1 configuration and ToF behavior are unchanged. */
    HAL_Delay(100);
    rtc_targeted_diagnostics();
#if RTC_SET_ON_BOOT
    RTC_DateTime set;
    if (RTC_ParseUTC(RTC_SET_UTC,&set)) {
        printf("[RTC] SET FAILED: RTC_SET_UTC must be YYYY-MM-DDTHH:MM:SSZ\n");
    } else {
        int set_rc=RTC_Set(&set);
        if (set_rc) { printf("[RTC] SET FAILED\n"); print_failure(set_rc); }
        else printf("[RTC] Time set explicitly; disable RTC_SET_ON_BOOT and rebuild\n");
    }
#endif
    RTC_DateTime dt; int rc=RTC_Get(&dt);
    if (!rc) {
        char text[21]; RTC_FormatUTC(&dt,text,sizeof(text));
        printf("[RTC] DS3231 UTC %s\n",text);
    } else print_failure(rc);
#endif
}
