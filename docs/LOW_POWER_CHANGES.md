# Low-Power Changes — Actual Edits Made


> All changes applied to `x-cube-n6-camera-capture` (STM32N6570-DK)

---

## 0. Ethernet PHY Power-Down — `ETH_PowerDown_Disable()`

**Location:** `Src/main.c` — function body at lines 584–700, called at line 1302

This function **was already present in the original code** (not written by us) but is part of our low-power configuration. It saves ~30–80 mW by putting the RTL8211F Ethernet PHY into deep sleep.

### What it does (3 layers of power saving):

**Layer 1 — MDIO power-down (the PHY itself):**
1. Enables ETH1 clocks temporarily (RGMII interface + MDIO bus)
2. Configures PA4 (MDIO) and PC1 (MDC) as alternate-function pins
3. Scans PHY addresses 0x00, 0x01, 0x1F, 0x02, 0x03 on the MDIO bus
4. When found, sets **BMCR bit 11** (Power Down) on the RTL8211F → PHY stops transmitting, PLL inside the PHY stops
5. Prints `[ETH] PHY found at addr 0x00  BMCR=0x1000  -> powering down`

**Layer 2 — GPIO isolation (prevents leakage):**
All 12+ Ethernet RGMII pins are reconfigured to ANALOG mode (High-Z):
- PA0–PA4, PA7, PA8
- PB5, PB6
- PC1, PC8
- PG11, PG13, PG14
- → No DC paths, no floating-input leakage on unpowered PHY outputs

**Layer 3 — Clock gating:**
`__HAL_RCC_ETH1_CLK_DISABLE()` + MAC/TX/RX clocks stopped
- ETH peripheral clocks gated → no clock tree toggling on unused peripheral bus

### Why it's relevant / Is it useful?

| Aspect | Detail |
|--------|--------|
| **Power saved** | ~30–80 mW (from ~1.09 W baseline → ~1.01–1.06 W) |
| **Cost** | ~200 µs at boot (MDIO transactions), permanently disabled after that |
| **Is it worth it?** | **Yes** — Ethernet is not used in this camera-capture project. The PHY draws power just sitting there doing nothing. Power-down costs nothing at runtime. |
| **Side effects** | If you later plug Ethernet, it won't work. This project has no Ethernet stack anyway. |

> **Verdict: Always enable for this project. Zero runtime downside, measurable power gain.**
## 1. `Inc/FreeRTOSConfig.h`

**Line 76 — tick rate comment updated**
```c
// Before:
#define configTICK_RATE_HZ  ((TickType_t)1000)    /* 1 ms tick */

// After:
#define configTICK_RATE_HZ  ((TickType_t)1000)   /* 1 ms tick — tickless idle suppresses SysTick during long sleep */
```
> Value unchanged (1000). We tried 100 Hz (10 ms tick) and reverted — it broke SD read benchmark and camera init.

---

## 2. `Inc/app_config.h`

### 2.1 — Removed `LP_GATE_PLL2/3/4` defines
```c
// Removed these 3 blocks entirely:
#ifndef LP_GATE_PLL2
#define LP_GATE_PLL2   0
#endif
#ifndef LP_GATE_PLL3
#define LP_GATE_PLL3   0
#endif
#ifndef LP_GATE_PLL4
#define LP_GATE_PLL4   0
#endif
```

### 2.2 — Removed `LP_DEBUG_PRINT_SLEEP_MS`
```c
// Removed:
#define LP_DEBUG_PRINT_SLEEP_MS   0
```

### 2.3 — Added `LP_DISABLE_LCD`
```c
// Inserted after LP_ENABLE_TICKLESS_IDLE:
#ifndef LP_DISABLE_LCD
#define LP_DISABLE_LCD    1   /* Power down unused LCD display + backlight */
#endif
```

### 2.4 — Consolidated trace flags under single `#if LP_TRACE_ALL` block
```c
// Before (repeated per-flag):
#if LP_TRACE_ALL
#define LP_TRACE_ETH_POWERDOWN  1
#else
#define LP_TRACE_ETH_POWERDOWN  0
#endif
#if LP_TRACE_ALL
#define LP_TRACE_GPIO_ISOLATION  1
...
// After (single block):
#define LP_TRACE_ALL    1
#if LP_TRACE_ALL
#define LP_TRACE_ETH_POWERDOWN        1
#define LP_TRACE_GPIO_ISOLATION       1
#define LP_TRACE_REGISTER_MODS       1
#define LP_TRACE_WAKEUP_LATENCY_US    0
#define LP_SUMMARY_AT_BOOT            1
#else
#define LP_TRACE_ETH_POWERDOWN        0
#define LP_TRACE_GPIO_ISOLATION       0
#define LP_TRACE_REGISTER_MODS       0
#define LP_TRACE_WAKEUP_LATENCY_US    0
#define LP_SUMMARY_AT_BOOT            0
#endif
```

---

## 3. `Src/main.c`

### 3.1 — Added LP trace print macros (new SECTION 2.5, lines 86–107)
```c
// Inserted new block:
#if LP_TRACE_ALL
#define LP_PRINT(flag, ...)  do { if ((flag)) printf(__VA_ARGS__); } while(0)
#define LP_TRACE_REG(mod_name, addr, before, after)                \
    do {                                                            \
        if (LP_TRACE_REGISTER_MODS)                                 \
            printf("[REG] %-20s @0x%08lX  0x%08lX -> 0x%08lX\n",   \
                   (mod_name), (unsigned long)(addr),                \
                   (unsigned long)(before), (unsigned long)(after)); \
    } while(0)
#else
#define LP_PRINT(flag, ...)           ((void)0)
#define LP_TRACE_REG(mod_name, addr, before, after)  ((void)0)
#endif
```

### 3.2 — Removed verbose LP summary block at boot
```c
// Removed:
#if LP_SUMMARY_AT_BOOT
    printf("[LP] Summary: ...\n");
    ...
#endif
```

### 3.3 — Simplified `LP_TicklessPreSleep` (lines 736–742)
```c
// Before: had LP_DEBUG_PRINT_SLEEP_MS + LP_GATE_PLL2/3/4 blocks inside
// After:
void LP_TicklessPreSleep(uint32_t xModifiableIdleTime)
{
    (void)xModifiableIdleTime;
#if LP_ENABLE_DEBUG_GPIO
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
#endif
}
```

### 3.4 — Simplified `LP_TicklessPostSleep` (lines 744–747)
```c
void LP_TicklessPostSleep(uint32_t xExpectedIdleTime)
{
    (void)xExpectedIdleTime;
}
```

### 3.5 — Added LCD power-down code (lines 1307–1321)
```c
#if LP_DISABLE_LCD
    {
        __HAL_RCC_GPIOQ_CLK_ENABLE();
        GPIO_InitTypeDef gpio_lcd = {0};
        gpio_lcd.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_lcd.Pull = GPIO_NOPULL;
        gpio_lcd.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_lcd.Pin = GPIO_PIN_6 | GPIO_PIN_3;
        HAL_GPIO_Init(GPIOQ, &gpio_lcd);
        HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_6, GPIO_PIN_RESET);   /* PQ6 = backlight OFF */
        HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_3, GPIO_PIN_RESET);   /* PQ3 = LCD_ONOFF OFF */
        printf("[LP] LCD display powered down\n");
    }
#endif
```

### 3.6 — Fixed SD read/write benchmark check (lines 817–854)
```c
// Before (both write and read blocks):
    if (st == HAL_OK && t1 > t0) {
    // if t1 == t0 (operation under 1ms), falls to else → FAIL

// After:
    if (st == HAL_OK) {
        uint32_t dt = (t1 > t0) ? (t1 - t0) : 1;
```

---

## 4. `Src/app_cam.c`

**Line ~205:**
```diff
- vTaskDelay(pdMS_TO_TICKS(1));
+ vTaskDelay(1);
```

---

## 5. `Src/app_thread.c`

**SD wait loop:**
```diff
- vTaskDelay(pdMS_TO_TICKS(1));
+ vTaskDelay(1);
```
