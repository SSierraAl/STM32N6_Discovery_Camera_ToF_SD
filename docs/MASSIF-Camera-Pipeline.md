# MASSIF camera pipeline and extension guide

This guide describes the overview/crop path used with `CAPTURE_MODE=4`, `CAM_BINNING=1`, `CAM_EXPOSURE_MODE=1` and `CAM_OVERVIEW_CROP=1` on `feature-cameraToF`.

## 1. Ownership and execution order

1. `sensor_task()` runs the ToF detector. An accepted ToF event starts a camera transaction through `Capture_RequestSnapshot()`.
2. `camera_task()` calls `CaptureSelectedPhoto()`. `CAM_CaptureFullFrame()` wakes the IMX335, discards the configured warmup frames, captures a stopped 2592 × 1944 YUYV image, invalidates the DMA destination cache, and returns the sensor to standby.
3. The LED is stopped before optical analysis. `CAM_LocalizeFrame()` finds foreground in the retained exposure and returns the object box, a bounded crop window and a connected-component count.
4. `SelectPhoto()` decides whether to discard, save a context image, save an overview/crop pair, or save a full-resolution/overview pair. This decision concerns storage after acquisition.
5. For a crop, `CopyNativeCrop()` copies native YUYV rows into the crop slot before any source resize. `DownsampleFirstImage()` then reduces the full source in place for the overview. Both files carry the same exposure timestamp.
6. For multiple components or an oversized crop, the full native frame is written first. After storage releases the buffer, the same source is reduced to make its overview. This requires no second full-frame allocation.
7. `SaveOverviewCropImage()` queues each record and waits until `storage_task()` finishes writing or recovery. The camera returns the completion semaphore after the transaction finishes. Only then does the sensor task resume its baseline/recovery/cooldown sequence.

The ToF orchestration is taken from the default branch `STM32N6_Discovery_Camera_ToF_SD` at commit `1e172171f71d2bca84c5fa94f2bc231903a67846`. Its trigger, baseline, latch and cooldown behavior is independent of the optical storage policy. Changes to optical detection should stay within the camera functions below.

The sensor task currently waits during camera processing and SD writes. Consequently, longer storage operations increase the interval between processed ToF observations. An insect that enters and leaves during this interval may be missed even when the ToF hardware is ranging.

## 2. Functions and data contracts

| Function | File | Inputs, outputs and reason for the boundary |
|---|---|---|
| `Capture_RequestSnapshot(timeout_ms)` | `Src/app_thread.c` | Queues one camera request and waits for completion. Timeout is in milliseconds. Buffer ownership must be released before a later transaction reuses it. |
| `CaptureSelectedPhoto(source, crop)` | `Src/app_thread.c` | Runs one complete acquisition, localization, selection and storage transaction. `source` is the native DMA slot. `crop` is the separate PSRAM workspace. |
| `CAM_CaptureFullFrame(buf, size, tick)` | `Src/app_cam.c`, declared in `Inc/app_cam.h` | `buf` must be 32-byte aligned and hold a native YUYV frame. `size` is bytes. Returns 1 for a valid stopped frame, otherwise 0. `tick` receives completion time in HAL milliseconds on success. |
| `CAM_LocalizeFrame(source, scratch, capacity, object, window, components)` | `Src/app_cam.c` | `source` is read-only native YUYV. `scratch` must be separate and aligned. `capacity` is bytes. `object` is the unpadded native box. `window` includes margin and alignment when it fits. `components` counts accepted connected regions. Returns a static diagnostic string. |
| `CAM_FindForeground(source, scratch, object, count)` | `Src/app_cam.c` | Built-in detector. Samples native luma, compares with the calibration grid and groups foreground. Writes a union box and region count. |
| `CAM_FitBurstWindow(box, capacity, window)` | `Src/app_cam.c` | Converts an unpadded native box to a padded, clipped, aligned YUYV window. Returns 1 if packed rows fit, otherwise 0. Failure leaves no valid crop output. |
| `CAM_DetectExternal(source, width, height, scratch, scratch_bytes, box)` | Public contract in `Inc/app_cam.h` | Project-supplied ML or other detector. Returns 1 for a valid box, 0 for no target, negative for an error. Input is full native YUYV. The project must provide and build an implementation when selecting this backend. |
| `SelectPhoto(object, crop, components, blank, tick, next)` | `Src/app_thread.c` | Applies the storage policy. Receives the unpadded box, padded crop, component count and positive blank classification. `next` is a candidate saved position. Returns a `PhotoAction_t`. |
| `CopyNativeCrop(source, crop, roi)` | `Src/app_thread.c` | Copies packed native rows from the same exposure used for localization. The validated ROI must fit the destination slot. |
| `DownsampleFirstImage(image)` | `Src/app_thread.c` | Box-filters YUYV in place using the compile-time divisor. Requires all native reads and crop extraction to have completed first. Compiled only when the divisor differs from 1. |
| `SaveOverviewCropImage(kind, image_kind, buffer, width, height, stamp)` | `Src/app_thread.c` | `kind` is a UART label, `image_kind` is the persistent record category, dimensions are pixels and `stamp` identifies the exposure. Blocks until storage releases the pixels. Returns 0 on success, nonzero on error. |
| `SD_StoreRawImage(...)` | `Src/app_thread.c` | Stores the header and pixel payload through the raw block writer and journal. The buffer is read-only until completion. |
| `image_kind_label(header)` | `raw_image_metadata.py` | Maps persistent category values to readable labels. Unknown and legacy values return an empty label. |
| `image_filename(header, index, suffix)` | `raw_image_metadata.py` | Generates a Windows-compatible PNG name from UTC, image ID, scan index, category and optional export suffix. |
| `snapshot_list_label(header, index)` | `raw_image_metadata.py` | Produces the viewer row in the order number, resolution, category, time, filename. |

### Coordinate rules

`CAM_BurstWindow` contains `x`, `y`, `width`, `height` in pixels of the oriented native 2592 × 1944 image. Right and bottom edges are exclusive. Validate nonzero dimensions and bounds before copying. Horizontal alignment preserves YUYV chroma pairs. The crop margin is applied only after localization. An object touching a sensor edge is different from a padded crop touching that edge.

`components` counts connected foreground regions. One insect with disconnected dark markings can generate multiple components. This count cannot establish insect identity or the number of individuals.

## 3. Built-in localization

The detector reads one luma sample near the center of each 16 × 16 native region. The resulting grid contains 162 × 122 cells. The first sufficiently bright capture after reboot initializes the empty-passage reference. Present the empty passage during that calibration. The reference is held in PSRAM and is lost on reboot.

Later frames are compared at the same spatial positions. A median brightness shift compensates for an overall illumination change. Cells sufficiently darker than the adjusted reference enter a four-neighbor connected-component search. Excessive global brightness shifts and widespread foreground return diagnostic outcomes rather than a fabricated bounding box.

A positive blank result requires no sampled changes exceeding the configured contrast threshold in either brightness direction. `no_connected_object` can still contain small or fragmented changes. It remains an uncertain observation and can be saved for diagnosis. Sampling cannot guarantee that a very small insect between grid samples will be detected.

The bench configuration contains an explicit exclusion for the measured fixed mark around native `(1264, 1200)`. It rejects only isolated components no larger than the configured size and entirely within the bounded vicinity. A larger connected object crossing that location remains eligible. This is a fixture-specific exclusion. It does not automatically teach arbitrary stationary insects into the background. Set `CAM_ROI_FIXED_MARK_MAX_SIZE=0U` after removing the mark, moving the background or changing the optics.

The sampled grid and connected-component traversal are O(N) in grid cells. Selection of a saved position is O(1). No NPU inference is executed by the built-in detector.

## 4. Storage selection and saved-position filtering

| Observation | Action |
|---|---|
| Empty reference captured or positive background match | Skip storage and reset saved-position continuity. |
| Missing or uncertain localization | Save `overview_uncertain` and reset continuity. |
| More than one accepted component | Save native full frame plus `overview_multi`. Reset continuity to avoid treating their union as one tracked insect. |
| One component, padded crop fits | Save overview and native crop unless the observation is a clear duplicate. |
| One component, crop exceeds capacity | Save native full frame plus `overview_large`, subject to saved-position filtering. |
| Center closer than the distance threshold to the last saved position | Skip writes if the reference is recent and the output class is unchanged. |
| Reference expired, output class changed, or an edge-truncated object becomes fully visible | Accept a fresh save even if displacement is small. |

Distance is measured from the last successfully saved position, not from every observed position. Small steps therefore accumulate until another photo is justified. Reverse movement works the same way as forward movement. The comparison uses squared distance, avoiding a square root.

A candidate reference is committed after the required writes succeed. A failed pair does not consume the opportunity to save at the next observation. No individual identity is assigned. A second insect very close to the previous center can be mistaken for a duplicate. Set the distance threshold to zero when validating recall.

## 5. Configuration map

All tunable parameters belong in `Inc/app_config.h`. Units below refer to native sensor coordinates unless stated otherwise.

| Parameter | Meaning and adjustment effect |
|---|---|
| `CAM_FIRST_IMAGE_DIVISOR` | 1 gives 2592 × 1944, 2 gives 1296 × 972, 3 gives 864 × 648, 4 gives 648 × 486. Changes the saved overview and CPU resize cost. Native acquisition and localization dimensions stay at 5 MP. |
| `CAM_SAVE_CROP_OVERVIEW` | Retained compatibility symbol. The current validation pipeline always includes an overview for an accepted target, regardless of this older switch. |
| `CAM_CROP_DETECTOR` | Selects `CAM_CROP_DETECTOR_WHITE` or `CAM_CROP_DETECTOR_EXTERNAL`. Selecting external requires a project implementation. |
| `CAM_FULL_WARMUP_FRAMES` | Frames discarded after waking before retaining the exposure. Lower values can reduce latency but require checking image stability on the board. |
| `CAM_EXPOSURE_VALUE` | Manual exposure in microseconds. Shorter exposures collect less light and reduce motion during integration. |
| `CAM_GAIN_VALUE` | Sensor gain in millidecibels. Higher values can increase visible noise. |
| `CAM_DYNAMIC_CROP_MARGIN` | Native pixels of context around the object. Larger values increase memory and storage requirements. |
| `CAM_BURST_CROP_BYTES` | Capacity shared by detector scratch and native crop. Any increase requires verifying linker placement and all PSRAM allocations. |
| `CAM_ROI_GRID_STEP` | Sampling spacing. Smaller values increase localization work and grid memory. Review queue-size and internal SRAM constraints before changing it. |
| `CAM_ROI_LUMA_DELTA` | Contrast threshold in 8-bit luma units. Lower values admit weaker targets and more noise. |
| `CAM_ROI_MIN_COMPONENT` | Minimum connected grid cells for a region. Larger values can lose small insects. |
| `CAM_ROI_WHITE_MIN` | Minimum median brightness for initial calibration. |
| `CAM_ROI_MAX_FOREGROUND_PCT` | Maximum foreground fraction before reporting widespread change. |
| `CAM_ROI_MAX_REFERENCE_SHIFT` | Allowed median illumination shift relative to calibration. |
| `CAM_ROI_FIXED_MARK_X`, `CAM_ROI_FIXED_MARK_Y` | Center of the known bench mark in native pixels. Must match the actual oriented image. |
| `CAM_ROI_FIXED_MARK_MAX_SIZE` | Maximum width and height of the isolated mark to exclude. Zero disables that exclusion. |
| `CAM_TRACK_MIN_DISTANCE_PX` | Center displacement required to save again. Zero disables distance-based suppression. |
| `CAM_TRACK_MAX_GAP_MS` | Maximum age of a saved position before accepting a fresh observation. |
| `APP_LOG_ANSI_COLORS` | 1 enables module-tag backgrounds, 0 emits plain bracket tags. CSV records remain plain. |

The working ToF configuration and `sensor_task()` should be validated as a unit. Baseline sample counts, adaptive refresh, holdoff, cooldown, latches, trigger thresholds and event policies interact. They are not camera tuning parameters.

## 6. Replacing the detector with ML

1. Keep native acquisition, buffer ownership and storage unchanged while introducing the backend.
2. Implement `CAM_DetectExternal()` in a source file included by the STM32CubeIDE project. Select `CAM_CROP_DETECTOR_EXTERNAL` in `app_config.h`.
3. Prepare the tensor from read-only YUYV, accounting for the model's channel order, quantization, resizing and letterbox padding. Keep tensor and model workspace within a measured memory budget. Scratch provided to the function is temporary and later overwritten by the crop copy.
4. Convert the returned model box to native image coordinates. Undo padding and scaling on each axis before applying camera bounds. Return the unpadded object rectangle. Do not add the crop margin in the model backend.
5. Return 1 only for a valid box, 0 for no target, and a negative value for inference failure. The camera validates dimensions and bounds and prepares the crop.
6. The current external API carries one box and no component count. Its observations use conservative association handling. To support multiple boxes or positive blank classification, extend the public contract, `CAM_LocalizeFrame()` and `SelectPhoto()` together. Do not infer an empty passage solely from an ML no-detection return.
7. Measure inference time, total capture-to-storage time, smallest-target recall and moving-target behavior on the board using the paired saved images.

To change storage decisions, edit `SelectPhoto()`. To change crop margins and bounds, edit `CAM_FitBurstWindow()`. To change only overview resolution, use the divisor. To change category names, update the record kind in `Inc/raw_image.h` and its mapping in `raw_image_metadata.py` together. Avoid assigning a new meaning to an existing persisted value.

## 7. Memory and SD format

The full YUYV source occupies 10,077,696 bytes (about 9.61 MiB). The crop/scratch slot reserves 2 MiB. The empty reference adds 19,764 bytes in PSRAM. Other firmware and storage allocations require additional space. The detector reuses the crop slot before crop extraction. One full source plus one crop slot is sufficient for the current sequence.

PSRAM reference storage is NOLOAD. Calibration fills the reference before the ready flag is set. Never read it based only on its apparent memory contents. DMA destinations require the existing cache and alignment discipline.

SD records retain the existing 64-byte header. `image_kind` occupies byte offset 44, previously reserved, and identifies the storage category. Old records with zero or unknown values retain legacy behavior in the viewer. The journal reserves storage before writing, and storage owns queued pixels until the completion semaphore is returned.

## 8. Terminal colors and viewer

`Inc/app_log.h` contains string-literal decorators used inside the existing `printf` call. Only the initial module tag receives a background and is immediately followed by the ANSI reset.

| Tag family | Display |
|---|---|
| `[CAM]`, `[IMX335]` | Black text on cyan |
| `[ToF]`, `[TOF ...]`, `[ADAPT]`, `[BASELINE ...]`, `[BTN]` | Black text on yellow |
| `[SD]`, `[SD META]` | White text on blue |

Enable ANSI/VT100 interpretation in the serial terminal. If escape codes appear literally, set `APP_LOG_ANSI_COLORS=0`. Exact-text log parsers for human-readable records may also need plain mode. `TOFEVT`, `TOFMISS` and other CSV output receive no color escapes.

The decorators allocate no buffers or state and add no extra printf calls. They add approximately 12 transmitted bytes per colored tag, roughly 1 ms on a 115200-baud 8N1 UART. Disable color for timing comparisons.

The viewer row is number, resolution, category, time and filename. A tooltip repeats the full row. The filename is also shown above the image and can be selected for copying. Use 100% native scale for sharpness inspection and Save Original for native PNG output. Fit-to-window changes display scale. Export at current scale changes output pixel dimensions.

## 9. Verification after a change

Build and flash with STM32CubeIDE. Start with the passage empty and check `empty_reference_saved`. Then test an empty trigger, one small object, movement below and above the saved-position threshold, disappearance and reappearance, a target touching an edge, an oversized target and two separated targets. Include a larger target crossing the fixture-mark location if the exclusion is enabled.

For each accepted event, verify paired timestamps and the expected category. Compare the crop with the same region of its overview. Confirm that blank and duplicate cases skip SD writes, and that baseline refresh, recovery and cooldown messages still follow the working ToF sequence.

Use `full_ms`, `detect_ms`, `select_ms`, `copy_ms`, `resize_ms`, `sd_ms` and `camera_task_total_ms` together. `detect_ms` measures localization, while `select_ms` includes diagnostic printing in the policy function. Neither is an end-to-end trigger latency. Full-resolution SD writes and CPU overview resizing can dominate the interval before the next ToF observation is processed.
