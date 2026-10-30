# STM32N6570-DK Low-Power Implementation — Surgical Steps

**Goal:** Reduce idle power from ~0.85 W to 0.50-0.75 W by making the CPU sleep
between ToF measurements. Every change below is minimal, isolated, and reversible.
Nothing else in the codebase is touched.

---

## Game Plan (4 Steps, Each Independent)

| Step | Change | Idle Power | Risk | Time |
|------|--------|-----------|------|------|
| **0** | Idle hook with WFI | ~0.80 W | None | 5 min |
| **1** | Tickless idle | ~0.75 W | Low | 5 min |
| **2** | Measurement GPIO | — | None | 10 min |
| **3** | ToF data-ready interrupt | ~0.75 W | Medium | 1 h |
| **4** | PLL/peripheral clock gating | ~0.65 W | Medium | 2 h |

Steps 0-2 don't change your detection logic at all. The ToF still polls every 10 ms.
Step 3 is optional — Steps 0-2 alone save most of the CPU power.

---

## Step 0: Idle Hook with WFI (5 min, reversible)

The CPU is currently running the FreeRTOS idle task at full speed (200 MHz) whenever
sensor_task, camera_task, and storage_task are blocked. Adding `__WFI()` in the idle
hook lets the CPU sleep between SysTick interrupts (every 1 ms).

### File: `Inc/FreeRTOSConfig.h` — Line 73

**Before:**
```c
#define configUSE_IDLE_HOOK                      0
```

**After:**
```c
#define configUSE_IDLE_HOOK                      1
```

### File: `Src/main.c` — at the end (before the closing `#ifdef`)

**Add this function anywhere in main.c:**
```c
/* Called by FreeRTOS idle task when no other task is ready.
   WFI stops the CPU clock until the next interrupt (SysTick every 1 ms,
   or an EXTI like USER button). Adds ~50 mW savings. */
void vApplicationIdleHook(void)
{
    __WFI();
}
```

### Revert:
- Set `configUSE_IDLE_HOOK` back to 0
- Delete the function

### Verify:
- System boots, all tasks run, ToF detects, camera captures, SD stores
- No change in behavior — just lower current
- Scope: the CPU core supply drops ~50 mA between sensor_task polls

---

## Step 1: FreeRTOS Tickless Idle (5 min, reversible)

Step 0 sleeps the CPU every 1 ms (on each SysTick), but SysTick keeps firing.
Tickless idle suppresses SysTick during extended sleep periods, so the CPU can
stay asleep for the full 10 ms between sensor_task polls.

### File: `Inc/FreeRTOSConfig.h`

**After `configUSE_IDLE_HOOK`, add:**
```c
#define configUSE_TICKLESS_IDLE              1
```

**Also add somewhere (e.g., near the tick rate):**
```c
#define configEXPECTED_IDLE_TIME_BEFORE_SLEEP 2
```

That's it. Two new `#define` lines. The FreeRTOS Cortex-M55 port in
`STM32Cube_FW_N6` already has `vPortSuppressTicksAndSleep()` — you don't need
to write it.

### What changes internally:
- The idle task calls `vPortSuppressTicksAndSleep()` instead of busy-looping
- SysTick is disabled before sleep, re-enabled on wake
- The port uses SysTick's current-value register to calculate elapsed time
- `xTaskGetTickCount()` includes sleep time — all timeouts correct
- The `vApplicationIdleHook()` from Step 0 still runs, but now it's called
  less often; the port handles the real WFI

### Revert:
- Comment out `configUSE_TICKLESS_IDLE`

### Verify:
- Same functional behavior as Step 0
- Scope: CPU now sleeps for ~9.8 ms between ToF polls (instead of waking every 1 ms)
- If the system hangs, the port might need a low-power timer for tick correction
  (rare on Cortex-M55 — SysTick continues running in Sleep mode)

---

## Step 2: Measurement GPIO (10 min, optional)

Add a GPIO toggle so you can measure sleep/wake timing on a scope.

### File: `app_config.h` — any section

**Add:**
```c
/* ================================================================
   LOW-POWER DEBUG — toggle GPIO on sleep entry/exit
   ================================================================ */
#define LOW_POWER_DEBUG_GPIO_ENABLE   0   /* 1 = enable, 0 = disabled */
#if LOW_POWER_DEBUG_GPIO_ENABLE
/* Use an unused GPIO on the DK board, e.g., PH3 (ARD D9 / CN18 pin 1).
   Verify it's not used elsewhere before enabling. */
#define LP_DEBUG_GPIO_PORT            GPIOH
#define LP_DEBUG_GPIO_PIN             GPIO_PIN_3
#endif
```

### File: `Src/main.c` — in `vApplicationIdleHook`

**After:**
```c
void vApplicationIdleHook(void)
{
#if LOW_POWER_DEBUG_GPIO_ENABLE
    HAL_GPIO_WritePin(LP_DEBUG_GPIO_PORT, LP_DEBUG_GPIO_PIN, GPIO_PIN_SET);   /* about to sleep */
#endif
    __WFI();
#if LOW_POWER_DEBUG_GPIO_ENABLE
    HAL_GPIO_WritePin(LP_DEBUG_GPIO_PORT, LP_DEBUG_GPIO_PIN, GPIO_PIN_RESET); /* woke up */
#endif
}
```

### Init (run once at boot in main.c):
```c
#if LOW_POWER_DEBUG_GPIO_ENABLE
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOH_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = LP_DEBUG_GPIO_PIN;
    HAL_GPIO_Init(LP_DEBUG_GPIO_PORT, &gpio);
    HAL_GPIO_WritePin(LP_DEBUG_GPIO_PORT, LP_DEBUG_GPIO_PIN, GPIO_PIN_RESET);
}
#endif
```

Toggle rate on scope tells you:
- Sleep duration (~10 ms between polls, or ~66 ms with Step 3)
- Wake duration (~200 µs per I2C read + detection)
- Confirms tickless idle is working (Step 1)

### Revert:
- Set `LOW_POWER_DEBUG_GPIO_ENABLE` to 0

---

## Step 3: ToF Data-Ready Interrupt (1 h, optional, replaces polling)

This removes the 10 ms periodic wake. Instead, the sensor blocks until the ToF
fires its INT pin. This is needed for deeper sleep modes (Stop) and reduces CPU
wakeups from 100/s to 15/s.

### Prerequisites:

1. **Find the ToF INT GPIO on the DK schematic.**
   The VL53L5CX on the CMW-IMX335 camera module has an INT output. It must be
   routed to a Discovery GPIO. If it's not connected, skip this step and keep
   the 10 ms polling — Steps 0-2 already save most of the CPU power.

2. **Test INT electrical connection:**
   - On a working system, probe the suspected INT pin with a scope
   - You should see a ~10 µs pulse at 15 Hz after each ToF measurement
   - If no pulse, the pin is wrong or the module doesn't route INT

### Changes:

### File: `Inc/app_config.h`

**Add:**
```c
/* ================================================================
   LOW-POWER SLEEP CONFIGURATION
   ================================================================ */
/* 0 = polling (10 ms loop, Steps 0-2 only)
   1 = ToF data-ready interrupt (Step 3) */
#define SLEEP_USE_TOF_INT            0

/* ToF INT GPIO — set these AFTER verifying the schematic */
#define TOF_INT_GPIO_PORT            GPIOX   /* CHANGE ME */
#define TOF_INT_GPIO_PIN             GPIO_PIN_0  /* CHANGE ME */
```

### File: `Inc/app_thread.h`

**Add after existing semaphore declarations:**
```c
/* ToF data-ready semaphore (created if SLEEP_USE_TOF_INT) */
extern SemaphoreHandle_t g_tof_data_ready_sem;
```

### File: `Src/app_thread.c` — in `IPC_Init()` (or nearby)

**Add semaphore creation:**
```c
#if SLEEP_USE_TOF_INT
    g_tof_data_ready_sem = xSemaphoreCreateBinary();
    if (g_tof_data_ready_sem == NULL) { printf("[FATAL] tof_data_ready_sem failed\n"); while (1); }
#endif
```

### File: `Src/app_thread.c` — replace the sensor_task polling loop

**Current loop (lines ~439-470 in app_thread.c):**
```c
    while (1) {
        if (g_sensor_state == SENSOR_STATE_PAUSED) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        // ... other checks (button refresh, dual sensor, etc) ...
        if (!VL53L5CX_Update()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        // ... detection logic ...
```

**Replace with:**
```c
    while (1) {
        if (g_sensor_state == SENSOR_STATE_PAUSED) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

#if SLEEP_USE_TOF_INT
        /* Block until ToF data-ready ISR gives the semaphore.
           CPU sleeps (tickless idle) during this wait. */
        xSemaphoreTake(g_tof_data_ready_sem, portMAX_DELAY);
        if (VL53L5CX_GetData() != 0) {
            /* Data read via I2C — run detection on fresh frame */
            /* VL53L5CX_Update() internally calls GetData, so we call Update */
            if (!VL53L5CX_Update()) continue;  /* Update = GetData + detection refresh */
        } else {
            continue;  /* I2C read failed, try again next INT */
        }
#else
        /* Original polling path (Steps 0-2) */
        if (!VL53L5CX_Update()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continxue;
        }
#endif

        // ... the rest of detection logic unchanged ...
        if (VL53L5CX_IsInsectDetected() && cooldown == 0) {
```

### File: `Src/app_thread.c` — add ISR

**Add this function (anywhere in the file):**
```c
#if SLEEP_USE_TOF_INT
/* ToF data-ready interrupt handler.
   Called on each completed VL53L5CX measurement.
   Gives the binary semaphore to unblock sensor_task. */
void TOF_INT_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    HAL_GPIO_EXTI_IRQHandler(TOF_INT_GPIO_PIN);
    xSemaphoreGiveFromISR(g_tof_data_ready_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* EXTI callback (called by HAL_GPIO_EXTI_IRQHandler) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TOF_INT_GPIO_PIN) {
        /* Clear the interrupt flag — EXTI line handled */
    }
}
#endif
```

### File: `Src/main.c` — configure EXTI (after ToF init)

**Add:**
```c
#if SLEEP_USE_TOF_INT
{
    GPIO_InitTypeDef gpio = {0};
    /* Enable GPIO clock for the INT pin's port */
    /* __HAL_RCC_GPIOX_CLK_ENABLE();  — UNCOMMENT with correct port */
    gpio.Pin = TOF_INT_GPIO_PIN;
    gpio.Mode = GPIO_MODE_IT_FALLING;   /* VL53L5CX INT is active low, pulse */
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TOF_INT_GPIO_PORT, &gpio);

    /* Set NVIC priority and enable */
    /* HAL_NVIC_SetPriority(EXTIx_IRQn, 5, 0);  — USE CORRECT EXTI NUMBER */
    /* HAL_NVIC_EnableIRQ(EXTIx_IRQn);           — USE CORRECT EXTI NUMBER */
    printf("[LP] ToF data-ready interrupt enabled on pin\n");
}
#endif
```

### Revert:
- Set `SLEEP_USE_TOF_INT` to 0
- All `#if SLEEP_USE_TOF_INT` blocks are compiled out

### Fallback (if INT pin is not routed):
Keep `SLEEP_USE_TOF_INT = 0` and keep the 10 ms polling loop. Steps 0-2 already
let the CPU sleep between polls. The difference between 10 ms polling and
interrupt-driven is only ~1% CPU duty cycle — measurable but not critical.

---

## Step 4: Clock Gating During Sleep (2 h, medium risk)

After Steps 0-2 are verified, add this to reduce PLL and peripheral clock power.

### File: `Inc/app_config.h`

**Add:**
```c
/* ================================================================
   LOW-POWER CLOCK GATING
   Disable unused PLLs/peripherals during idle sleep.
   ================================================================ */
#define SLEEP_GATE_PLL2             0   /* 1 = disable PLL2 (CSI/DCMIPP) during sleep */
#define SLEEP_GATE_PLL3             0   /* 1 = disable PLL3 during sleep */
#define SLEEP_GATE_PLL4             0   /* 1 = disable PLL4 (USB) during sleep */
#define SLEEP_GATE_SDMMC2           0   /* 1 = gate SDMMC2 clock during sleep */
#define SLEEP_GATE_UART             0   /* 1 = gate USART1 clock during sleep */
```

### File: `Src/main.c` — in `vApplicationIdleHook`

**Replace:**
```c
void vApplicationIdleHook(void)
{
#if LOW_POWER_DEBUG_GPIO_ENABLE
    HAL_GPIO_WritePin(LP_DEBUG_GPIO_PORT, LP_DEBUG_GPIO_PIN, GPIO_PIN_SET);
#endif

#if SLEEP_GATE_PLL2
    HAL_RCCEx_DisablePLL2();   /* CSI/DCMIPP not needed in idle */
#endif
#if SLEEP_GATE_PLL3
    HAL_RCCEx_DisablePLL3();
#endif
#if SLEEP_GATE_PLL4
    HAL_RCCEx_DisablePLL4();
#endif

    __WFI();

#if SLEEP_GATE_PLL2
    HAL_RCCEx_EnablePLL2();
    /* Wait for PLL2 ready — ~50 µs */
    while (__HAL_RCC_GET_PLL2_READY_FLAG() == 0) {}
#endif
#if SLEEP_GATE_PLL3
    HAL_RCCEx_EnablePLL3();
    while (__HAL_RCC_GET_PLL3_READY_FLAG() == 0) {}
#endif
#if SLEEP_GATE_PLL4
    HAL_RCCEx_EnablePLL4();
    while (__HAL_RCC_GET_PLL4_READY_FLAG() == 0) {}
#endif

#if LOW_POWER_DEBUG_GPIO_ENABLE
    HAL_GPIO_WritePin(LP_DEBUG_GPIO_PORT, LP_DEBUG_GPIO_PIN, GPIO_PIN_RESET);
#endif
}
```

**Important:** PLL2 has a startup delay of ~50-100 µs. This adds to the wake
latency. If the ToF data-ready interrupt fires, the PLL must be ready before
the I2C read. Verify with the measurement GPIO.

---

## Files Changed Summary

| File | Lines Changed | Changes |
|------|--------------|---------|
| `Inc/FreeRTOSConfig.h` | 3 | Enable idle hook, tickless idle, expected idle time |
| `Src/main.c` | 10-30 | Idle hook function, optional GPIO init, optional EXTI config |
| `Inc/app_config.h` | 10-20 | Debug GPIO, ToF INT pin, clock gating flags |
| `Inc/app_thread.h` | 2 | Semaphore extern declaration |
| `Src/app_thread.c` | 15-30 | Sensor task loop patch, optional ISR function |

**Not touched:**
- `vl53l5cx_detection.c` — untouched unless Step 3 is enabled
- `app_cam.c` — untouched (camera standby already works)
- `platform.c` — untouched (I2C unchanged)
- `i2c_arbiter.c` — untouched
- `Makefile` — untouched
- Linker script — untouched

---

## Validation Checklist

After EACH step, confirm:

- [ ] System boots (GREEN LED on, console shows "System READY")
- [ ] ToF initializes and learns baseline
- [ ] Detection triggers correctly at known targets
- [ ] Camera wakes, captures, stores to SD
- [ ] Returns to idle after capture
- [ ] Multiple capture cycles (10+) without crash
- [ ] After Steps 0-1: measured idle current < before

If any step fails, revert that step's changes — the system goes back to exactly
how it was. No cascade, no side effects.

---

## Ethernet Note

You said you're disabling Ethernet on the DK board. The Ethernet PHY on the
Discovery board (typically at specific GPIOs for RMII) draws ~100-200 mW in idle.
Disabling it is separate from the MCU sleep changes — it's a board-level power
reduction that stacks on top of the MCU savings:

| Configuration | Estimated Idle Power |
|--------------|---------------------|
| Current (Ethernet ON, no sleep) | ~0.85 W |
| Ethernet OFF, no sleep | ~0.70 W |
| Ethernet OFF + Steps 0-2 | ~0.60-0.65 W |
| Ethernet OFF + Steps 0-4 | ~0.50-0.60 W |
| Ethernet OFF + Stop mode (future) | ~0.25-0.35 W |

---

## Reference

- `Documentation/LOW_POWER_MODE4_ANALYSIS.md` — previous deeper analysis
- STM32N6 RM (RM0482) — Sleep/Stop modes, PLL config
- AN5946 — Low-power modes on STM32N6
- UM3300 — Discovery kit with STM32N657X0 MCU
- ST example: `x-cube-n6-ai-power-measurement` — Discovery Sleep during camera capture
