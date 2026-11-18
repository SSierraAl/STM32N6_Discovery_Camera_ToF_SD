# MASSIF insect imaging firmware

Autonomous insect-passage imaging for the MASSIF project. A multizone time-of-flight (ToF) sensor monitors the trap. When it detects an event, the STM32N6 illuminates the passage, captures a Sony IMX335 image and decides what to save to microSD. The unit runs independently after programming. The Windows viewer scans the card and exports images.

## Hardware and firmware

| Component | Role |
| --- | --- |
| STM32N6570-DK Discovery | STM32N6 application, FreeRTOS tasks and XSPI PSRAM. The inherited CubeIDE project name contains `nucleo`. |
| B-CAMS-IMX / Sony IMX335 | MIPI CSI-2 input to DCMIPP. Full field is 2592 × 1944. The firmware stores ISP-processed YUYV pixels. |
| VL53L5CX | I²C ranging and event detection. The checked-in camera profile uses one sensor and a 4 × 4 grid. |
| WS2812 illumination | Lighting during acquisition, driven by timer and DMA. |
| microSD and DS3231 | Raw image records over 4-bit SDMMC2. The RTC supplies UTC when valid. |

## Capture modes

| Use | Build configuration | Output |
| --- | --- | --- |
| Autonomous field imaging **(default on `feature-cameraToF`)** | `CAPTURE_MODE=4`, `CAM_OVERVIEW_CROP=1` | One 5 MP exposure per accepted ToF event. The optical result determines whether to save an overview, an overview with a native crop, or an overview with the native full frame. Calibration, blank and duplicate observations can produce zero records. |
| On-demand bench capture | `CAPTURE_MODE=0`, `TEST_TOF_MODE=0` | Press USER (PC13) for one 2592 × 1944 full-field record with the default mode-0 settings. Useful for camera, light and card checks. |
| Callback-batch capture | `CAPTURE_MODE=4`, `CAM_OVERVIEW_CROP=0`, `CAM_NATIVE_ROI=1` | Two separately timed 640 × 768 frames of the configured fixed window by default. |
| ToF commissioning | `CAPTURE_MODE=4`, `TEST_TOF_MODE=1` | ToF console events and board LED feedback. USER requests a fresh ToF baseline. |

The [firmware guide](docs/firmware-guide.md#operating-modes) covers the remaining capture variants, task flow and configuration dependencies.

## Build and first capture

1. Check out `feature-cameraToF`. In STM32CubeIDE, choose **File → Import → General → Existing Projects into Workspace** and select the [CubeIDE project directory](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/STM32CubeIDE/NUCLEO-N657X0-Q). Leave **Copy projects into workspace** unchecked so the linked sources resolve in the checkout.
2. Select the profile in [`Inc/app_config.h`](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Inc/app_config.h), build the project and use the included ST-LINK Debug configuration. The profile is compiled into the image. Rebuild and reload it after changing the header.
3. Use a dedicated capture microSD card. The firmware writes raw sectors starting at block 3072 and writes its journal immediately before that area. Reserve the card for capture data.
4. Open the target UART at **115200 baud, 8N1**. In on-demand mode, press USER and look for a saved snapshot. In the default mode, keep the passage empty during ToF baseline learning and through the **first accepted camera exposure**. That exposure calibrates the optical reference. Look for `empty_reference_saved`, then introduce a test target and look for the camera selection and SD save messages.
5. When writing has completed, stop the firmware and move the card to the host. Scan it and verify the result in the viewer.

## Inspect captures on Windows

From a Python environment on Windows, install the viewer dependencies and run it with administrator access for raw drive access.

```powershell
python -m pip install PySide6 numpy opencv-python
python .\SD_Image_Viewer.py
```

Select the physical microSD, **Refresh**, then **Scan Snapshots**. Inspect images at native scale and use **Download All** or **Extract Selected** to export PNGs. **Format SD** erases scanned STM32 records and resets the append journal. Confirm the selected physical disk before using it.

## Documentation

| Guide | Use |
| --- | --- |
| [Firmware and operating modes](docs/firmware-guide.md) | Follow startup, ToF, camera/ROI and SD from the entry points to the stored records. Includes configuration and bench checks. |
| [Camera pipeline and extension guide](docs/MASSIF-Camera-Pipeline.md) | Work on optical localization, crop selection, memory ownership, or the external detector interface. |
| [Optical analysis](docs/reference/optical-analysis.md) | Compare the 6, 8 and 10 mm f/8 lenses. |
| [Python image filter](docs/reference/python-image-filter.md) | Understand the separate host-side image-filtering experiment. [Spanish reference](docs/reference/python-image-filter-es.md). |
| [Engineering archive](docs/archive/README.md) | Find earlier designs, bench observations, experiments and field notes. |

The [firmware project](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main) contains `Src/`, `Inc/`, `Lib/` and the ST package. The repository root contains the [SD viewer](SD_Image_Viewer.py), the [ToF zone monitor](vl53l5cx_zone_monitor.py) and the [ToF datalogger](vl53l5cx_datalogger.py). The latter tools consume serial telemetry from the board.
