# Project MASSIF 
**Monitoring Automatisé et Systèmes de Surveillance Intelligents de la biodiversité des insectes dans les écosystèmes Forestiers français**

**Developers:** 
*   Dr. Sebastian Sierra Alarcon
*   Rafael Hernan Valverde

**Établissement coordinateur:** INRAE  
**Établissements partenaires:** CNRS | INP Toulouse | Sorbonne Université | Muséum national d'Histoire naturelle | ONF  

---

## 1. System Operating Modes
The firmware configuration is governed by `app_config.h`, which acts as the ultimate truth for the system's operational behavior. The `CAPTURE_MODE` macro selects the core capture logic.

### Legacy & Testing Modes
*   **Mode 0 (ON-DEMAND):** A manual mode triggered by the USER button (PC13). The ToF sensor is completely disabled. The camera performs a full initialization, warm-up (11 frames), and captures a single 2592×1944 (5MP) image before shutting down. It provides the highest image quality but carries a 10-second total cycle latency.
*   **Mode 1 (CONTINUOUS):** An intermediate ToF-triggered mode where the camera pipeline runs continuously at 1296×972. It captures a single frame via a CPU `memcpy`, which creates a ~700ms memory bottleneck.
*   **Mode 2 (BATCH):** Similar to Mode 1, but captures multiple frames per detection. It suffers from a known I2C bus conflict (returning error `ret=-7`) and pipe-restart tearing mid-burst. 

### Production Mode
*   **Mode 4 (CALLBACK-BATCH):** The optimized, recommended production mode for insect detection. The camera is initialized once at boot and parked in standb. Upon ToF trigger, the system wakes the camera, starts a double-buffered pipeline, and captures a burst of frames (default 4) at 1296×972. 
    *   **Zero-Copy DMA:** To prevent memory bottlenecks, Mode 4 keeps the DCMIPP hardware pipeline running continuously during the burst. The DMA address registers are dynamically reprogrammed on-the-fly, dropping each frame directly into its final PSRAM allocation slot without a single CPU `memcpy`.
    *   **Performance:** Capture time for 3-4 frames is roughly ~130ms, with zero image tearing because the pipe never restarts mid-burst.

---

## 2. FreeRTOS Thread Architecture
The firmware operates on a single-core Cortex-M55 running a FreeRTOS port with a 1 kHz tick rate. All tasks are statically allocated, avoiding dynamic heap usage to ensure long-term stability in the field. 

| Task Name | Priority | Role & Responsibilities |
| :--- | :--- | :--- |
| `main_thread` | **IDLE + 5** | The startup gatekeeper. Initializes board hardware, clocks, SD card (with 5 retries), and the camera callback before setting `system_ready = 1` and blocking forever. |
| `sensor_task` | **IDLE + 4** | The master orchestrator. Owns the VL53L5CX ToF sensor, handles the 5-second boot stabilization window, and actively polls for targets. Upon detection, it triggers the entire capture sequence via `Capture_RequestSnapshot()`. |
| `camera_task` | **IDLE + 3** | Owns the IMX335 burst acquisition. It wakes the camera, manages the zero-copy DMA hardware frame events, packages the frame pointers into a struct, and sends them to the storage queue. |
| `storage_task` | **IDLE + 2** | Consumes storage jobs and executes blocking SD card multi-block writes via `SD_StoreRawImage()`. It manages block cursor advancement and auto-recovery if an SD timeout occurs. |
| `btn_thread` | **IDLE + 3** | *(Mode 0 Only)* Owns the manual button polling loop, completely bypassing the IPC queues used in autonomous modes. |

---

## 3. Inter-Process Communication (IPC) & Contracts
Because the trigger path requires ultra-low latency, tasks do not share complex memory blocks. Instead, they operate a strict queue-and-semaphore pipeline created during boot via `IPC_Init()`.

*   **`camera_cmd_queue` (Depth 4):** Produced by `sensor_task`, consumed by `camera_task`. Transmits the initial `CAM_CMD_SNAP` trigger to begin the hardware wake-up sequence.
*   **`storage_cmd_queue` (Depth 8):** Produced by `camera_task`, consumed by `storage_task`. Contains `StorageCmd_t` objects, which pass the immutable PSRAM buffer address, geometry, and snapshot ID of the completed frames.
*   **`camera_ready_sem` (Binary):** Given by `camera_task` back to `sensor_task` to signal that the camera hardware has finished the burst (or failed) and has returned to low-power standby.
*   **`storage_done_sem` (Counting):** Given by `storage_task` to `sensor_task`. It provides one token per successfully persisted frame, signaling that the SD card is free.

## 4. Concurrency & Synchronization Rules
To prevent bus collisions and ensure predictable behavior in the wild, the system enforces rigid ownership rules:
1.  **The Startup Gate:** No operational task acts until `main_thread` asserts the `system_ready` flag, preventing initialization race conditions.
2.  **Strict Serialization (`g_capture_busy`):** The `sensor_task` asserts a busy gate during an active capture/write cycle. The ToF sensor pauses, and new insect detections are ignored until the SD card finishes writing the current burst. 
3.  **I2C Arbiter:** A global recursive FreeRTOS mutex (`g_i2c1_mutex`) serializes all traffic on the shared I2C1 bus, completely preventing the camera (address `0x1A`/`0x36`) and ToF sensor (address `0x29`) from corrupting each other's configuration registers.
4.  **Data Cache Coherency:** Because hardware DMA operates independently of the CPU cache, `SCB_InvalidateDCache` is strictly enforced before CPU or SD access, ensuring the MCU reads the actual DMA frame data and not stale cache.