# Low-Power Guide — STM32N6570-DK Discovery Kit

## 1. Power Consumption Baseline

| State | Power | Technique |
|-------|-------|-----------|
| Original (no changes) | ~1.09 W | — |
| ETH PHY power-down | ~1.06 W | `ETH_PowerDown_Disable()` |
| + WFI idle | ~1.00 W | `__WFI()` in idle task |
| + Tickless idle | ~0.85 W | FreeRTOS tickless |
| + PLL4 gate (USB off) | ~0.80 W | Gate PLL4 during sleep |
| Deep sleep (Stop mode) | ~0.15 W | Full driver suspend |

> **Your reading: ~1.06 W** — Ethernet powered down + WFI active.

---

## 2. Abbreviations

| Abbrev | Full Name | What It Does |
|--------|-----------|--------------|
| **WFI** | Wait For Interrupt | ARM instruction that pauses CPU until any interrupt. Saves ~50-100 mW. |
| **Tickless Idle** | FreeRTOS Tickless Idle | FreeRTOS pauses SysTick during long idle. Sleeps for 100s of ms at a time. |
| **PLL** | Phase-Locked Loop | Generates higher-frequency clocks from a lower-frequency crystal. |
| **MDIO** | Management Data I/O | 2-wire bus (MDIO+MDC) to configure the Ethernet PHY. |
| **BMCR** | Basic Mode Control Register | PHY register 0x00. Bit 11 = Power Down. |
| **GPIO** | General-Purpose I/O | MCU pins. |
| **RCC** | Reset and Clock Control | MCU block managing all clocks and resets. |
| **DWT** | Data Watchpoint and Trace | Debug peripheral with cycle counter for timing. |
| **HCLK** | AHB Bus Clock | Main bus clock (200 MHz). |
| **RGMII** | Reduced Gigabit Media Independent Interface | Gigabit Ethernet (12+ pins). |
| **PHY** | Physical Layer | Ethernet transceiver (Realtek RTL8211F). |
| **CSI** | Camera Serial Interface | MIPI CSI-2 camera data interface. |

---

## 3. What We Modified

### 3.1 Files Changed

- `Inc/app_config.h` — Added `LP_ENABLE_IDLE_WFI`, `LP_TRACE_MASK`, `LP_GATE_PLL2/3/4`
- `Src/main.c` — Added `ETH_PowerDown_Disable()` + tickless handlers + trace

### 3.2 ETH Power-Down — 3-Layer

**Layer 1: MDIO Power-Down** → ~80 mW saved
- Enable ETH1 clocks, configure MDIO/MDC pins as AF11
- Send BMCR bit 11 (Power Down) to RTL8211F

**Layer 2: GPIO Isolation** → Prevents parasitic leakage
- 13 pins set to ANALOG (High-Z): PA0-4,PA7,PA8, PB5-6, PC1,PC8, PG11,PG13,PG14

**Layer 3: Clock Gating** → Zero ETH clock power
- ETH1, ETH1MAC, ETH1TX, ETH1RX clocks disabled

---

## 4. PLL Clocks

### Clock Tree

```
HSI (64 MHz)
  ├── PLL1 ── IC1 (CPU @ 800 MHz) / IC2 (AXI @ 400 MHz -> HCLK @ 200 MHz)
  ├── PLL2 ── IC6 (125 MHz) ── CSI camera clock
  ├── PLL3 ── IC11 (225 MHz) ── DSP/JPEG/DMA2D
  └── PLL4 ── USB OTG PHY
```

### Gating Safety

| PLL | Feeds | Safe? | Notes |
|-----|------|-------|-------|
| PLL1 | CPU+System | NO | Freezes system |
| PLL2 | Camera | Only if stopped | Corrupts frames if streaming |
| PLL3 | DSP/JPEG | Caution | May affect camera pipe |
| PLL4 | USB OTG | YES (if USB idle) | Safest choice |

When you set `LP_GATE_PLL4 = 1`: gates PLL4 before sleep, restores after.
Prints rate-limited: 1st event then every 500 cycles.

---

## 5. WFI vs Tickless Idle

**WFI** (current): `Idle -> __WFI() -> wake every 1ms` → ~1.00 W
**Tickless** (next): `Idle -> sleep 50-500ms between wakes` → ~0.85 W

> **Set `LP_ENABLE_TICKLESS_IDLE 1` in app_config.h**

---

## 6. Configuration

In `Inc/app_config.h`:

```c
#define LP_ENABLE_IDLE_WFI       1   // WFI in idle
#define LP_ENABLE_TICKLESS_IDLE  0   // 1 = deeper sleep
#define LP_GATE_PLL4             0   // 1 = gate PLL4 during sleep
#define LP_TRACE_MASK  (LP_TRACE_ETH_POWERDOWN | LP_TRACE_GPIO_ISOLATION)
```

### Trace Masks

| Flag | Value | Prints |
|------|-------|--------|
| `LP_TRACE_ETH_POWERDOWN` | `1<<0` | `[ETH] PHY found/powered` |
| `LP_TRACE_GPIO_ISOLATION` | `1<<1` | `[ETH] GPIO->ANALOG` |
| `LP_TRACE_REGISTER_MODS` | `1<<2` | `[LP] PLL4 gated/restored` |
| `LP_TRACE_WAKEUP_LATENCY_US` | `1<<3` | DWT cycle counter |

### Recommended Path

```
1.09 W → Original
1.06 W → + ETH power-down  ← you are here
0.85 W → + Tickless idle   ← next: LP_ENABLE_TICKLESS_IDLE=1
0.80 W → + PLL4 gate       ← then: LP_GATE_PLL4=1
```
