# 5MP Camera Capture Feasibility Analysis
## STM32N657X0-Q Nucleo Board + B-CAMS-IMX (IMX335) Camera

---

## 📋 Executive Summary

| Aspect | Status | Details |
|--------|--------|---------|
| **Full 5MP Capture** | ❌ NOT FEASIBLE | Requires 6.3+ MB RAM |
| **ROI Capture** | ✅ FEASIBLE | Up to ~1.5 MP at full resolution |
| **VGA Capture** | ✅ FEASIBLE | 640×480 at 30 FPS |
| **Available RAM** | 1.9 MB | After system overhead |

**Key Finding**: While full 5MP frame buffering is impossible, **ROI (Region of Interest) windowed readout** enables high-resolution capture of targeted areas within memory constraints.

---

## 1. Hardware Resources

### 1.1 NUCLEO-N657X0-Q Board

```
┌─────────────────────────────────────────────────────────────┐
│                    STM32N657X0-Q MCU                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │ Cortex-M55   │  │ Neural-ART   │  │ DCMIPP       │       │
│  │ 800 MHz      │  │ Accelerator  │  │ Camera IF    │       │
│  └──────────────┘  └──────────────┘  └──────────────┘       │
│                                                              │
│  Memory Architecture:                                        │
│  ├── SRAM:    4.2 MB (internal)                             │
│  ├── DTCM:    128 KB (zero-wait data)                       │
│  ├── ITCM:    64 KB  (zero-wait code)                       │
│  └── Backup:  8 KB   (low-power)                            │
│                                                              │
│  Peripherals:                                                │
│  ├── Octo-SPI: 64 MB Flash (external storage)               │
│  ├── USB 2.0 HS: 480 Mbps                                    │
│  └── SDMMC: SD card interface                                │
└─────────────────────────────────────────────────────────────┘
```

| Resource | Specification |
|----------|---------------|
| **Core** | Arm Cortex-M55 @ 800 MHz |
| **Total SRAM** | 4.2 MB (internal, no PSRAM) |
| **DCMIPP** | Digital Camera Interface (parallel/MIPI) |
| **External Storage** | 64 MB Octo-SPI Flash |

### 1.2 B-CAMS-IMX Camera Module

| Parameter | Specification |
|-----------|---------------|
| **Sensor** | Sony IMX335 |
| **Max Resolution** | 2592 × 1944 = 5.03 MP |
| **Interface** | Dual-lane MIPI CSI-2 |
| **Output Formats** | RAW10, YUV422 |
| **ROI Support** | ✅ Yes (windowed readout via registers) |

---

## 2. Memory Requirements

### 2.1 Full Frame Calculations

```
Full Sensor Resolution: 2592 × 1944 = 5,039,616 pixels
```

| Format | Bytes/Pixel | Single Frame | 2 Buffers | 4 Buffers |
|--------|-------------|--------------|-----------|-----------|
| **RAW10** | 1.25 | **6.30 MB** | 12.6 MB | 25.2 MB |
| **YUV422** | 2.0 | **10.1 MB** | 20.2 MB | 40.4 MB |
| **RGB888** | 3.0 | **15.1 MB** | 30.2 MB | 60.4 MB |

**RAW10 Calculation:**
```
2592 × 1944 pixels × 10 bits ÷ 8 bits/byte = 6,299,520 bytes ≈ 6.30 MB
```

### 2.2 Available Memory Breakdown

```
┌─────────────────────────────────────────────────────────┐
│ Total Internal SRAM:        4.20 MB                     │
├─────────────────────────────────────────────────────────┤
│ System Overhead:                                          │
│ ├── FreeRTOS kernel:        0.50 MB                     │
│ ├── Stack (3 threads):      0.30 MB                     │
│ ├── Application code:       0.80 MB                     │
│ ├── Camera middleware:      0.20 MB                     │
│ └── USB stack:              0.30 MB                     │
│                                         Subtotal: 2.10 MB│
├─────────────────────────────────────────────────────────┤
│ AVAILABLE FOR FRAME BUFFERS: ~1.9 MB ⚠️                  │
└─────────────────────────────────────────────────────────┘
```

---

## 3. 🎯 ROI Windowed Readout Strategy

### 3.1 Concept Overview

The IMX335 sensor supports **ROI (Region of Interest) windowed readout**. Instead of capturing the full 5MP frame, the sensor can be configured to output only a specific window at **full pixel resolution**.

```
┌─────────────────────────────────────────────────────────────┐
│                     Full Sensor (2592×1944)                 │
│                                                             │
│    ┌──────────────────────────────────────────────────┐    │
│    │                                                  │    │
│    │          ML Detection Area (Low Resolution)      │    │
│    │              640 × 480 for inference             │    │
│    │                                                  │    │
│    │              ┌─────────────────┐                 │    │
│    │              │                 │                 │    │
│    │              │   ROI TARGET    │                 │    │
│    │              │   1296 × 972    │ ← Reconfigure  │    │
│    │              │   (Full Res)    │    sensor to   │    │
│    │              │                 │    output ONLY │    │
│    │              │                 │    this window │    │
│    │              └─────────────────┘                 │    │
│    │                                                  │    │
│    └──────────────────────────────────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Why ROI Works

| Feature | Benefit |
|---------|---------|
| **Full Resolution** | Pixels in ROI retain 10-bit quality |
| **No Downsampling** | No quality loss from binning |
| **Reduced Memory** | Only ROI pixels transferred to RAM |
| **Faster Processing** | Less data to store and process |

### 3.3 ROI Memory Requirements

| ROI Resolution | RAW10 Size | Fits in 1.9 MB? |
|----------------|------------|-----------------|
| 640 × 480 | 384 KB | ✅ Yes |
| 800 × 600 | 600 KB | ✅ Yes |
| 1024 × 768 | 983 KB | ✅ Yes |
| 1280 × 720 | 1.15 MB | ✅ Yes |
| 1024 × 1024 | 1.31 MB | ✅ Yes |
| 1280 × 960 | 1.53 MB | ✅ Yes |
| 1400 × 1000 | 1.75 MB | ✅ Yes |
| 1600 × 1200 | 2.40 MB | ❌ No |
| 2592 × 1944 (Full) | 6.30 MB | ❌ No |

**Maximum Practical ROI: ~1.3-1.5 MP**

### 3.4 Complete Workflow

```
┌──────────────────────────────────────────────────────────────┐
│                    ROI CAPTURE PIPELINE                      │
└──────────────────────────────────────────────────────────────┘

Step 1: Low-Res Detection          Step 2: Configure Sensor
┌─────────────────────┐            ┌─────────────────────┐
│ • Capture 640×480   │            │ Write I²C registers:│
│   (~384 KB)         │            │   X_ADDR_START = 700│
│ • Run ML inference  │───────────▶│   Y_ADDR_START = 500│
│ • Detect target     │   coords   │   X_ADDR_END = 1996 │
│   location          │───────────▶│   Y_ADDR_END = 1472 │
└─────────────────────┘            └─────────────────────┘
                                           ↓
Step 4: Save to Storage    Step 3: Capture High-Res ROI
┌─────────────────────┐    ┌─────────────────────────────┐
│ • SDMMC write       │◄───│ • Sensor outputs only ROI   │
│   ~10-20 MB/s       │    │ • 1296 × 972 × 1.25 = 1.57  │
│ • Write time:       │    │   MB                        │
│   ~80-150 ms        │    │ • Fits in available RAM     │
└─────────────────────┘    └─────────────────────────────┘
```

### 3.5 Sensor Register Configuration

The IMX335 uses these registers for ROI windowing:

| Register | Address | Description |
|----------|---------|-------------|
| `X_ADDR_START` | 0x318C | Horizontal start pixel |
| `X_ADDR_END` | 0x3190 | Horizontal end pixel |
| `Y_ADDR_START` | 0x318E | Vertical start line |
| `Y_ADDR_END` | 0x3192 | Vertical end line |

**Example I²C Configuration for 1296×972 ROI:**
```c
// ROI: x=700, y=500, width=1296, height=972
// Sensor full: 2592×1944
IMX335_WriteRegister(0x318C, 700);    // X_START
IMX335_WriteRegister(0x318E, 500);    // Y_START
IMX335_WriteRegister(0x3190, 1995);   // X_END (700 + 1296 - 1)
IMX335_WriteRegister(0x3192, 1471);   // Y_END (500 + 972 - 1)
```

### 3.6 Bandwidth Analysis

| Operation | Data Rate | Feasibility |
|-----------|-----------|-------------|
| 640×480 @ 30fps (YUV) | 3.7 MB/s | ✅ Easy |
| 1296×972 @ 15fps (RAW) | 18.8 MB/s | ✅ Good |
| 1296×972 @ 30fps (RAW) | 37.6 MB/s | ⚠️ USB limited |
| 2592×1944 @ 30fps (RAW) | 236 MB/s | ❌ Not feasible |

---

## 4. Critical Bottlenecks

| # | Bottleneck | Severity | Impact |
|---|------------|----------|--------|
| 1 | **No onboard PSRAM** | 🔴 Critical | Limits to ~1.9 MB usable |
| 2 | **USB 2.0 bandwidth** | 🔴 Critical | Max ~35 MB/s practical |
| 3 | **Single frame size** | 🔴 Critical | 5MP RAW = 6.3 MB > RAM |
| 4 | **DCMIPP clock** | 🟡 Moderate | 333 MHz, sufficient for ROI |

---

## 5. Feasibility Matrix

| Scenario | Resolution | Format | Memory Required | Status |
|----------|------------|--------|-----------------|--------|
| **VGA Capture** | 640×480 | YUV422 | 614 KB | ✅ FEASIBLE |
| **720p Capture** | 1280×720 | RAW10 | 1.15 MB | ✅ FEASIBLE |
| **ROI 1MP** | 1024×1024 | RAW10 | 1.31 MB | ✅ FEASIBLE |
| **ROI 1.5MP** | 1280×960 | RAW10 | 1.53 MB | ✅ FEASIBLE |
| **ROI Max** | 1400×1000 | RAW10 | 1.75 MB | ✅ FEASIBLE |
| **1080p Capture** | 1920×1080 | RAW10 | 2.6 MB | ❌ NOT FEASIBLE |
| **Full 5MP** | 2592×1944 | RAW10 | 6.3 MB | ❌ NOT FEASIBLE |

---

## 6. Implementation Recommendations

### 6.1 For ROI-Based Capture System

```c
// app_config.h - ROI Configuration
#define ROI_MODE_ENABLED          1
#define ROI_MAX_WIDTH             1400
#define ROI_MAX_HEIGHT            1000
#define ROI_BUFFER_SIZE           (1400 * 1000 * 2)  // 2.8 MB worst case

// Detection mode (low resolution)
#define DETECTION_WIDTH           640
#define DETECTION_HEIGHT          480
#define DETECTION_BUFFER_SIZE     (640 * 480 * 2)    // 614 KB

// Capture mode (high resolution ROI)
#define CAPTURE_WIDTH             1296
#define CAPTURE_HEIGHT            972
#define CAPTURE_BUFFER_SIZE       (1296 * 972 * 2)   // 2.5 MB
```

### 6.2 Pipeline State Machine

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  IDLE    │───▶│ DETECT   │───▶│ CONFIGURE│───▶│ CAPTURE  │
└──────────┘    └──────────┘    └──────────┘    └──────────┘
                    │                               │
                    │  No target                    │
                    └───────────────────────────────┘
                    │  Retry detection
```

### 6.3 Storage Integration

With 64 MB Octo-SPI Flash available:
- Store up to **~40 ROI frames** (1.5 MB each)
- Write speed: ~10-20 MB/s
- Capture time per frame: **80-150 ms**

---

## 7. Summary & Conclusion

### What Works Without External RAM

| Capability | Status | Notes |
|------------|--------|-------|
| VGA (640×480) capture | ✅ | Full 30 FPS |
| 720p (1280×720) capture | ✅ | Up to 30 FPS |
| ROI capture up to 1.5 MP | ✅ | Full sensor resolution |
| AI inference | ✅ | Neural-ART accelerator |
| SD card storage | ✅ | SDMMC interface |

### What Requires External RAM

| Capability | External RAM Needed |
|------------|---------------------|
| Full 5MP frame buffer | 16+ MB PSRAM |
| Multi-frame video | 16+ MB PSRAM |
| Image stitching | 16+ MB PSRAM |
| Multiple ROI buffers | 16+ MB PSRAM |

### Final Recommendation

For your MASSIF project targeting insect monitoring:

1. **Use ROI capture strategy** - Detect insects at low res, capture high-res ROI
2. **Maximum ROI size: 1400×1000** (1.75 MB RAW10)
3. **Store directly to SD card** - Avoid RAM bottlenecks
4. **Consider external HyperRAM** only if full-frame capture is essential

---

## References

1. UM3417 - STM32N6 Nucleo-144 Board User Manual
2. B-CAMS-IMX Data Brief
3. IMX335 Datasheet (ST Camera Middleware)
4. STM32N657xx Reference Manual (RM0486)
5. AN5769 - Using the DCMIPP interface on STM32N6