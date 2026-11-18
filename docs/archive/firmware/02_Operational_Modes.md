# Project MASSIF 
**Monitoring Automatisé et Systèmes de Surveillance Intelligents de la biodiversité des insectes dans les écosystèmes Forestiers français**

---

## 1. Operational Modes Overview
The STM32N6 firmware relies on two validated capture topologies, dictated by the `CAPTURE_MODE` macro. Rather than relying on a generic filesystem, both modes output raw, self-describing YUV422 records directly to physical SD blocks to guarantee deterministic write times.

*   **Mode 0 (On-Demand Snapshot):** The manual calibration and testing path. It yields the absolute highest image quality (full 5MP) but carries a long cycle time.
*   **Mode 4 (Callback-Batch):** The autonomous production path. Optimized for field deployment, it balances ultra-low latency, zero-copy memory transfers, and standby power management to capture moving insects without image tearing.

---

## 2. Mode 0: On-Demand Snapshot (Manual Validation)
Mode 0 acts as the reference path for validating optics, manual exposure, and the SD write path before enabling autonomous operation. The Time-of-Flight (ToF) sensor is completely disabled to dedicate all system resources to a single, high-resolution frame.

### 2.1. Trigger & Resolution
*   **Trigger:** Hardware USER button (PC13).
*   **Resolution:** 2592 × 1944 (Full 5MP).
*   **Buffer Size:** ~10 MB per frame.

### 2.2. Execution Flow & Logic
Because latency is not a concern in manual mode, the camera is kept completely unpowered until the button is pressed. 
1.  **Wake:** The button press immediately turns on the WS2812 LED illumination.
2.  **Warm-up:** The camera undergoes a full cold initialization. The pipeline is started, and 11 warm-up frames are captured and discarded to allow the sensor's analog gain to stabilize.
3.  **Capture & Persist:** A single 5MP frame is captured into PSRAM. The CPU then blocks while dumping this 10 MB payload to the SD card.
4.  **De-initialization:** The LED shuts off, and the camera is completely de-initialized and powered down.

### 2.3. Thread Orchestration
Mode 0 intentionally bypasses the complex Inter-Process Communication (IPC) queues used in autonomous modes. It relies entirely on a single thread (`btn_thread` running at `IDLE + 3` priority) which synchronously owns the button polling, the camera initialization, and the SD card writing.

---

## 3. Mode 4: Callback-Batch (Autonomous Production)
Mode 4 is the pinnacle of the firmware's architecture. It resolves the conflicting goals of battery-friendly idling and sub-millisecond responsiveness. It captures 4 continuous frames per insect detection with zero image tearing.

### 3.1. Trigger & Resolution
*   **Trigger:** VL53L5CX ToF sensor insect detection.
*   **Resolution:** 1296 × 972 (2x2 Hardware Binned).
*   **Buffer Size:** 4 frames × 2.5 MB = 10 MB allocated in PSRAM.

### 3.2. Deep Logic: Zero-Copy DMA & Double-Buffering
Previous iterations of this firmware suffered from "tearing" (glitched images) because the CPU had to stop the camera pipeline, copy the image to a save buffer (`memcpy`), and restart the pipeline. Mode 4 eliminates this bottleneck entirely:

1.  **Continuous Pipeline:** Once woken by the ToF sensor, the DCMIPP hardware pipeline starts and *never stops* during the burst. 
2.  **Hardware Ping-Pong:** During the 11-frame warm-up, the hardware alternates writing frames into two scratch buffers (`capture_buf` and `save_buf`). 
3.  **Dynamic Address Reprogramming:** Exactly two frames before a "keeper" frame is needed, the CPU executes `HAL_DCMIPP_PIPE_SetMemoryAddress()`. This instantly redirects the hardware's DMA destination. 
4.  **Zero CPU Overhead:** By the time the frame arrives, the hardware DMA drops the raw pixels directly into their final target slot in the `batch_buf`. The CPU performs zero copying; it simply invalidates the Data Cache (`SCB_InvalidateDCache`) to ensure the SD card reads fresh memory.

### 3.3. Thread Orchestration & IPC Pipeline
Mode 4 relies on a strictly prioritized, single-core FreeRTOS pipeline. Tasks communicate via static queues and semaphores to ensure the camera outpaces the SD card without dropping data.

| Task | Priority | Orchestration Role |
| :--- | :--- | :--- |
| `sensor_task` | **High** (`IDLE + 4`) | The master trigger. Runs continuously polling the ToF sensor. Upon detection, it asserts a `g_capture_busy` lock (ignoring new insects), turns on the LED flash, and sends a `CAM_CMD_SNAP` to the camera queue. |
| `camera_task` | **Med** (`IDLE + 3`) | Wakes the IMX335 from standby (taking ~35ms) and manages the Zero-Copy DMA hardware events. Once the 4-frame burst lands in PSRAM, it sends 4 address pointers to the storage queue and immediately signals the sensor that the camera hardware is free. |
| `storage_task` | **Low** (`IDLE + 2`) | Consumes the pointers and executes heavy, blocking multi-block writes to the SD card. It manages its own auto-recovery if the SD card times out. Once a frame is successfully saved, it releases a counting semaphore token back to the sensor task. |

### 3.4. Synchronization Safeguards
*   **I2C Arbiter:** Both the ToF sensor and the Camera share the physical I2C1 bus. A global recursive FreeRTOS mutex acts as a strict gatekeeper, wrapping all I2C calls so the camera's wake-up routine never collides with the ToF's baseline polling.
*   **Capture Cooldown:** Once the `storage_task` confirms all 4 frames are safely written to the SD card, the `sensor_task` drops the busy lock, turns off the LED, and enforces a mandatory 1-second (30 ToF frames) cooldown to prevent re-triggering on the exact same insect.