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

static int lock(void)
{
    return g_i2c1_mutex &&
           xSemaphoreTake(g_i2c1_mutex, pdMS_TO_TICKS(20)) == pdTRUE;
}

static int read_reg(unsigned reg, uint8_t *data, unsigned n)
{
    last_hal = HAL_I2C_Mem_Read(&hi2c1, RTC_ADDRESS, reg,
                                I2C_MEMADD_SIZE_8BIT, data, n, 20);
    last_error = HAL_I2C_GetError(&hi2c1);
    return last_hal == HAL_OK ? 0 : -1;
}

static int write_reg(unsigned reg, uint8_t *data, unsigned n)
{
    last_hal = HAL_I2C_Mem_Write(&hi2c1, RTC_ADDRESS, reg,
                                 I2C_MEMADD_SIZE_8BIT, data, n, 20);
    last_error = HAL_I2C_GetError(&hi2c1);
    return last_hal == HAL_OK ? 0 : -1;
}

#if RTC_ENABLE
static void print_failure(int rc)
{
    if (rc == -3) {
        printf("[RTC] I2C1 mutex unavailable\n");
    } else if (rc == -1) {
        printf("[RTC] Read failed at 0x68: HAL=%u error=0x%08lX\n",
               last_hal, (unsigned long)last_error);
    } else if (rc == -2) {
        printf("[RTC] Time invalid/unset (control=0x%02X status=0x%02X). "
               "Provision once with RTC_SET_ON_BOOT=1.\n",
               (unsigned)last_control, (unsigned)last_status);
    } else {
        printf("[RTC] Invalid date/time argument\n");
    }
}
#endif

int RTC_Get(RTC_DateTime *dt)
{
    uint8_t r[16];
    if (!dt) return -4;
    if (!lock()) return -3;

    /* One coherent burst: START latches calendar; includes control/status. */
    int rc = read_reg(0, r, sizeof(r));
    if (!rc) {
        last_control = r[14];
        last_status = r[15];
    }
    xSemaphoreGive(g_i2c1_mutex);

    if (rc) return -1;
    if ((r[15] & 0x80U) || (r[14] & 0x80U)) return -2;
    return RTC_Decode(r, dt) == 0 ? 0 : -2;
}

int RTC_Set(const RTC_DateTime *dt)
{
    uint8_t r[7], control = 0, status = 0;
    if (RTC_Encode(dt, r)) return -4;
    if (!lock()) return -3;

    int rc = read_reg(14, &control, 1);
    control &= 0x7fU; /* EOSC=0: retain time on battery too. */
    if (!rc) rc = write_reg(14, &control, 1);
    if (!rc) rc = write_reg(0, r, 7);
    if (!rc) rc = read_reg(15, &status, 1);
    if (!rc) {
        /* Clear OSF only after an explicit valid time write. Writing 1 to
           alarm flags leaves them unchanged, including concurrent alarms. */
        status = (status & 0x7fU) | 3U;
        rc = write_reg(15, &status, 1);
    }

    xSemaphoreGive(g_i2c1_mutex);
    return rc;
}

RTC_Stamp RTC_CaptureStamp(void)
{
    RTC_Stamp s = {0, HAL_GetTick(), 0};
#if RTC_ENABLE
    RTC_DateTime dt;
    int rc = RTC_Get(&dt);
    if (rc == 0) {
        s.unix_seconds = RTC_ToUnix(&dt);
        s.valid = 1;
    } else {
        /* Avoid flooding UART on every capture if the clock becomes invalid.
           RTC_Init() already reports the detailed startup condition. */
        static uint8_t capture_error_reported = 0;
        if (!capture_error_reported) {
            print_failure(rc);
            capture_error_reported = 1;
        }
    }
#endif
    return s;
}

void RTC_Init(void)
{
#if RTC_ENABLE
#if RTC_SET_ON_BOOT
    RTC_DateTime set;
    if (RTC_ParseUTC(RTC_SET_UTC, &set)) {
        printf("[RTC] SET FAILED: RTC_SET_UTC must be YYYY-MM-DDTHH:MM:SSZ\n");
    } else {
        int set_rc = RTC_Set(&set);
        if (set_rc) {
            printf("[RTC] SET FAILED\n");
            print_failure(set_rc);
        } else {
            printf("[RTC] UTC provisioned; restore RTC_SET_ON_BOOT=0 and rebuild\n");
        }
    }
#endif

    RTC_DateTime dt;
    int rc = RTC_Get(&dt);
    if (!rc) {
        char text[21];
        RTC_FormatUTC(&dt, text, sizeof(text));
        printf("[RTC] UTC %s\n", text);
    } else {
        print_failure(rc);
    }
#endif
}
