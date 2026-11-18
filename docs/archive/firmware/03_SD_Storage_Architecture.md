# Project MASSIF 
**Monitoring Automatisé et Systèmes de Surveillance Intelligents de la biodiversité des insectes dans les écosystèmes Forestiers français**

---

## 1. Storage Architecture Rationale
Writing multi-megabyte high-speed bursts to a standard microSD card using a traditional FAT/FileX filesystem introduces unpredictable overhead, metadata fragmentation, and non-deterministic write latencies. 

To guarantee that the camera pipeline is never starved or blocked, Project MASSIF entirely bypasses conventional filesystems. Instead, it utilizes a custom raw-block writer via `HAL_SD_WriteBlocks()`. Images are serialized into self-describing binary records written sequentially to physical SD card sectors, maximizing the continuous throughput of the SDMMC2 4-bit interface.

---

## 2. Raw Block Record Layout
Each capture session begins writing at a safe offset to preserve any potential bootloaders (`SD_SNAP_BASE_BLOCK` = 3072, ~1.5MB into the card). Every image record consists of a strict 64-byte metadata header followed immediately by the raw YUV422 pixel payload. 

### Header Binary Layout (`sd_image_header_t`)
Values are written as native STM32 little-endian 32-bit unsigned integers.

| Byte Offset | Field | Description |
| :--- | :--- | :--- |
| `0` | `magic` | `0x49444745` ("EDGE"). Used by the Python decoder to find valid images. |
| `4` | `width` | Pixel width (e.g., 1296 or 2592). |
| `8` | `height` | Pixel height (e.g., 972 or 1944). |
| `12` | `pixel_format` | Defines color space (0 = YUV422). |
| `16` | `data_size` | Pure payload byte count, excluding the 64-byte header. |
| `20` | `timestamp` | `HAL_GetTick()` execution time (milliseconds since boot). |
| `24` | `checksum` | XOR checksum of every payload byte for data integrity validation. |
| `28` | `snap_id` | Sequential firmware snapshot identifier. |
| `32..59` | `reserved` | Zero-filled. Reserved for future metadata (e.g., ToF distance, temp). |
| `60..63` | `tail` | Padding to ensure the header aligns perfectly to 64 bytes. |

---

## 3. The Batched Write Algorithm & The `STA=0x5000` Problem
Writing a 2.5MB frame requires pushing ~4,920 blocks to the SD card. Pushing them block-by-block is too slow; pushing them all at once crashes the card. The firmware uses a meticulously tuned **Batched Multi-Block** strategy.

### The Algorithm Logic
1. **Staging:** The CPU packs the 64-byte header and the first chunk of DMA image data into a dedicated 32KB PSRAM staging buffer (`sd_batch_buf`).
2. **Batching:** Writes are strictly executed in batches of **64 blocks** (`SD_BATCH_WRITE_BLOCKS`).
3. **Cache Coherency:** Before the SD controller takes over, the CPU executes `SCB_CleanDCache` to flush the staging buffer from the CPU cache to physical memory.
4. **The Hybrid Wait (Solving the `STA=0x5000` Error):** 
   * *The Problem:* SDXC cards aggressively report "TRANSFER Ready" via `CMD13` while their internal NAND flash is still busy programming. If the STM32 sends the next batch too early, the card panics and throws a Data Command Response Timeout (`STA=0x5000`).
   * *The Solution:* The firmware polls `CMD13` until the card claims it is ready. Then, it intentionally executes a hard `vTaskDelay(15ms)`. This 15ms inter-batch recovery gap acts as a crucial safety buffer, allowing the card's internal NAND controller to finish its erase/program cycle.

---

## 4. Error Recovery & Fail-Safes
SD cards are inherently volatile in field conditions (temperature drops, power spikes). The `storage_task` is designed to try to handle those situations.

If `HAL_SD_WriteBlocks()` fails (e.g., timeout or CRC error):
1. The firmware triggers `SD_Reinit()`.
2. It aborts the current transfer, physically resets the SDMMC2 hardware clock, and re-initializes the SD card interface.
3. The block cursor (`g_sd_img_base_block`) is rolled back.
4. The exact same image buffer is retried. 
5. If it succeeds, no data is lost. If it fails again, the frame is dropped, the queue is cleared, and the system prepares for the next insect detection.

---

## 5. Host-Side Decoding: Python Architecture
Because the SD card has no FAT filesystem, Windows/Mac will prompt you to "Format the drive" when inserted. **Do not format it.** Instead, the companion desktop software (`SD_Image_Viewer.py`) reads the raw physical sectors.

### Python Decoder Logic
1. **Physical Drive Access:** The script requires Administrator privileges to open physical block devices directly (e.g., `\\.\PhysicalDrive2` on Windows or `/dev/diskX` on Unix).
2. **Variable-Stride Scanner:** The script seeks to block 3072 and reads 64 bytes. It checks for the `0x49444745` magic tag.
   * If found, it reads the `data_size` to know exactly how many bytes to skip to find the *next* image header. This allows the card to safely store a mix of 5MP and 1.3MP frames dynamically.
3. **Bulk Read:** Instead of reading block-by-block (which takes ~120s per image in Python), the script executes a single, massive OS-level read for the entire payload size, reducing read times to ~1 second.
4. **Color Conversion:** The raw payload is mathematically unpacked from YUV422 (YCbCr) into an sRGB NumPy array and displayed in the Tkinter GUI, where it can be exported as standard PNG/BMP files.

---

## 6. Logic Flow Diagram: Memory to Desktop

```text
[STM32N6 Hardware]                                          [SD Card]
      │                                                         │
      ├─► DCMIPP Hardware DMA drops frame to `batch_buf`        │
      │                                                         │
      ├─► `camera_task` sends pointer to `storage_cmd_queue`    │
      │                                                         │
      ├─► `storage_task` wakes up, reads pointer                │
      │                                                         │
      ├─► Calculates XOR checksum & builds 64-byte header       │
      │                                                         │
      ├─► PACKS header + pixels into 32KB `sd_batch_buf`        │
      │                                                         │
      ├─► `SCB_CleanDCache` (Flush cache to physical memory)    │
      │                                                         │
      ├─► SDMMC2 Hardware DMA Transfer (64 Blocks) ────────────►│ Writes to Sectors
      │                                                         │
      ├─► Poll CMD13 for Card Ready state ◄─────────────────────┤ Card replies "Ready"
      │                                                         │
      ├─► Mandatory 15ms `vTaskDelay` (NAND internal wait)      │
      │                                                         │
      └─► Loop until frame is complete...                       │
                                                                │
====================== REMOVE CARD & INSERT TO PC ======================
                                                                │
[Desktop PC / Python Decoder]                                   │
      │                                                         │
      ├─► Open \\.\PhysicalDrive (Admin Mode) ◄─────────────────┤
      │                                                         │
      ├─► Seek to Block 3072. Read 64 bytes.                    │
      │                                                         │
      ├─► Assert Magic == 0x49444745?                           │
      │                                                         │
      ├─► Extract W, H, Size. Execute Bulk Read ◄───────────────┤ Pulls Raw Payload
      │                                                         │
      ├─► Convert YUV422 to RGB via NumPy matrices              │
      │                                                         │
      └─► Render to Screen / Export as PNG