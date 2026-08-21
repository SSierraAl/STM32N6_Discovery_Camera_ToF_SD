# Project MASSIF 
**Monitoring Automatisé et Systèmes de Surveillance Intelligents de la biodiversité des insectes dans les écosystèmes Forestiers français**

---

## 1. Camera Middleware (CMW) & Pipeline Architecture
The Camera Middleware (CMW) acts as the software abstraction layer bridging the Sony IMX335 sensor, the STM32's Image Signal Processor (ISP), and the Digital Camera Interface Pixel Pipeline (DCMIPP) hardware. 

The architecture strictly separates control and data:
*   **Control Plane (I2C1):** CMW manages sensor configuration (exposure, gain, standby state) via a shared I2C bus at 100kHz.
*   **Data Plane (CSI-2):** The IMX335 streams raw pixel data over a high-speed MIPI CSI-2 bus directly into the DCMIPP hardware.
*   **Processing & Transport:** The DCMIPP mathematically packs the data into YUV422 format (2 bytes per pixel) and utilizes its own DMA engine to push frames to external PSRAM without CPU intervention.

---

## 2. Event-Driven Hardware Pipeline

[IMX335 Sensor] --(CSI-2)--> [DCMIPP Hardware] --(DMA Engine)--> [External PSRAM]
                                    |
                                    +--> VSYNC Interrupt (Start of Frame)
                                    |
                                    +--> FRAME EVENT Interrupt (End of Frame)

Instead of using CPU-blocking software delays, the pipeline is strictly interrupt-driven. 
*   `HAL_DCMIPP_PIPE_VsyncEventCallback()` fires when the sensor begins exposing the first row.
*   `HAL_DCMIPP_PIPE_FrameEventCallback()` fires only when the DCMIPP DMA has successfully written the final pixel of the frame into memory. 
*   The application polling loop purely tracks the volatile `g_frame_event_count` incremented by this ISR to ensure complete data integrity before reading.

---

## 3. Exposure, Gain & Binning Orchestration (`app_config.h`)
Image quality and geometry are controlled by compile-time macros mapped directly to CMW overrides.

*   **Manual Mode Enforcement:** `CAM_EXPOSURE_MODE = 1` is strictly required in autonomous modes to guarantee deterministic results. 
*   **The ISP Overwrite Quirk:** The ST ISP library automatically overwrites exposure settings when the pipeline starts; therefore, the application code must explicitly re-apply exposure via I2C after the pipe is started.
*   **Exposure Limits:** `CAM_EXPOSURE_VALUE` must remain strictly less than or equal to the `SNAP_FPS` frame period in microseconds to prevent rolling shutter mathematical overflow.
*   **Hardware Binning:** Setting `CAM_BINNING = 1` applies decimation purely in hardware via the DCMIPP. 
*   **Write Optimization:** This hardware binning shrinks the DMA output from ~10 MB (2592x1944) down to ~2.5 MB (1296x972), heavily optimizing the SD card write times.

---

## 4. Mode 4 Execution Logic: The Zero-Copy DMA Sequence
In the recommended `CALLBACK-BATCH` mode, the CMW executes a precise 7-step hardware orchestration to capture a burst of frames without a single CPU memcpy]. 

1.  **Wake Sequence:** The IMX335 streaming register is written via I2C to wake it from hardware standby, followed by a mandatory 35ms delay to allow the analog power rails to stabilize.
2.  **Start Hardware Pipe:** `CMW_CAMERA_DoubleBufferStart()` starts the DCMIPP once, configuring it with two alternating scratch destinations (`capture_buf` and `save_buf`).
3.  **Override ISP:** The configured manual mode, exposure, and gain settings are re-transmitted over I2C to override the ISP defaults.
4.  **Hardware Warm-up:** The system waits for `CALLBACK_WARMUP_FRAMES` complete frame events while the hardware DMA endlessly overwrites the scratch buffers and the sensor's analog gain stabilizes.
5.  **Dynamic Address Reprogramming:** Exactly two frames before a required output frame, the CPU calls `HAL_DCMIPP_PIPE_SetMemoryAddress()`. 
6.  **Zero-Copy Execution:** Because the DCMIPP alternates two internal address registers each frame, this redirects the hardware DMA engine to drop the incoming raw pixels directly into their final slot in `batch_buf[frame_index]`.
7.  **Cache Invalidation:** The CPU executes zero copy commands and merely calls `SCB_InvalidateDCache_by_Addr()` on the completed output buffer so the SD card's DMA reads the actual hardware data from physical memory instead of stale CPU cache.
8.  **Shutdown:** After the requested frames are successfully captured, the DCMIPP pipe is stopped once, and the IMX335 is returned to I2C standby.

Because the pipeline never stops or restarts mid-batch during this sequence, sensor synchronization is perfectly maintained, ensuring all captured frames are perfectly sharp and free of tearing.