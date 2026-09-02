/**
 * @file    app_sleep.h
 * @brief   STOP-mode sleep API for SLEEP-SNAPSHOT (CAPTURE_MODE = 5).
 *
 *   One boot, many snapshots: the MCU parks in STOP mode and wakes on
 *   the USER button (PC13 = PWR wake pin WKUP3, high level). See
 *   SLEEP_MODE_PLAN.md for the power budget and state machine.
 */
#ifndef APP_SLEEP_H
#define APP_SLEEP_H

#include <stdint.h>

/**
 * @brief  One-time wake-source setup (call at boot, before system_ready).
 *         Enables PWR + WKUP3 (PC13, HIGH polarity) and arms the
 *         WAKEUP_PIN interrupt at a FreeRTOS-safe priority.
 * @return 0 on success
 */
int  App_Sleep_Init(void);

/**
 * @brief  Enter STOP sleep; blocks until the USER button wakes the CPU.
 *         Call from a task. On return the PLL clocks are restored and
 *         the button is currently pressed (wait for release yourself).
 *         The function also handles the optional power-downs defined
 *         in app_config.h (user LEDs, NOR deep power-down) and restores
 *         them on wake.
 */
void App_Sleep_Enter(void);

#endif /* APP_SLEEP_H */
