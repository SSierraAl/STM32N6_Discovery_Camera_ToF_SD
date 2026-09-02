/**
 * @file    app_sleep.c
 * @brief   STOP-mode sleep for SLEEP-SNAPSHOT (CAPTURE_MODE = 5).
 *
 *   The system boots once (SD + camera + WS2812), the camera is parked
 *   in sensor STANDBY, and the CPU sits in STOP mode. The USER button
 *   (PC13, active HIGH with external pull-down) is the dedicated PWR
 *   wake pin WKUP3: a HIGH level on PC13 exits STOP and the CPU resumes
 *   on HSI until SystemClock_Config() re-locks the PLLs.
 *
 *   STOP mode retains RAM, PSRAM, all peripheral registers and the
 *   FreeRTOS scheduler state, so on wake the button task simply runs
 *   CAM_StandbySnap() + the normal SD save path — no subsystem is
 *   re-initialized.
 *
 *   Power notes: see SLEEP_MODE_PLAN.md (sleep budget + manual
 *   measurement protocol).
 */

#include "app_config.h"
#include "app_sleep.h"
#include "main.h"
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_xspi.h"
#include "ws2812.h"

#include "FreeRTOS.h"
#include "task.h"

/* Defined in main.c. De-static'ed specifically so the STOP-wake path
   can restore the PLL clocks after every wake. */
extern void SystemClock_Config(void);

/**
 * @brief  One-time setup of the wake source (boot, before system_ready).
 *
 *   - enables the PWR peripheral clock
 *   - enables wake pin WKUP3 (PC13) with HIGH-level polarity
 *   - arms the WAKEUP_PIN interrupt at a FreeRTOS-safe priority
 *
 * @return 0 on success
 */
int App_Sleep_Init(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();

    /* USER button: active HIGH with external pull-down -> wake on HIGH. */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH);

    /* The ISR (WAKEUP_PIN_IRQHandler in stm32n6xx_it.c) only clears the
       PWR flag — no RTOS calls — but keep the priority in the
       FreeRTOS-safe range regardless. */
    HAL_NVIC_SetPriority(WAKEUP_PIN_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(WAKEUP_PIN_IRQn);

    return 0;
}

/**
 * @brief  Enter STOP sleep and block until the USER button wakes the CPU.
 *
 *   Call from a task. Sequence:
 *     wait button release -> optional power-downs (WS2812 zero frame,
 *     user LEDs, NOR deep power-down) -> console drain -> D-cache clean
 *     -> SysTick IRQ off + WFI (STOP) -> on wake: NOR restore, SysTick
 *     on, SystemClock_Config() (PLL re-lock), D-cache invalidate.
 */
void App_Sleep_Enter(void)
{
    /* 1) Never sleep while the button is held: WKUP3 is level-triggered,
          a HIGH at WFI time would wake up immediately. */
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* 2) Drop everything that can be powered down for the nap.
          WS2812_TurnOff() is a BLOCKING DMA send -> LEDs guaranteed
          off before the clocks stop. */
    WS2812_TurnOff();
#if SLEEP_LEDS_OFF
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_RED);
#endif
#if SLEEP_NOR_DEEP_PD
    BSP_XSPI_NOR_EnterDeepPowerDown(0);
#endif

    /* 3) Drain the console (blocking UART) before the clocks stop. */
    printf("[SLEEP] STOP mode: press USER button (PC13) to wake\n");
    fflush(stdout);
    HAL_Delay(SLEEP_UART_SETTLE_MS);

    /* 4) Push dirty cache lines into memory (retained in STOP). */
    SCB_CleanDCache();

    /* 5) Freeze the RTOS tick, then STOP.
          No FreeRTOS API is called between PRIMASK=1 and the wake, so
          the scheduler state comes back exactly as it was. */
    NVIC_DisableIRQ(SysTick_IRQn);
    __set_PRIMASK(1);
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* ---- CPU is back here: button pressed (PC13 HIGH), on HSI clock ---- */

    /* 6) Restore the NOR flash if it was deep-powered-down. */
#if SLEEP_NOR_DEEP_PD
    BSP_XSPI_NOR_LeaveDeepPowerDown(0);
#endif

    /* 7) Re-enable the tick, then restore the full PLL clocks. */
    NVIC_EnableIRQ(SysTick_IRQn);
    __set_PRIMASK(0);
    SystemClock_Config();

    /* 8) Nothing external writes memory while we sleep, but invalidate
          anyway for a known-good cache state. */
    SCB_InvalidateDCache();
}
