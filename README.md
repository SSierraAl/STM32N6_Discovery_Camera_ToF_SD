# MASSIF insect imaging firmware

MASSIF uses a time-of-flight (ToF) sensor to watch for insects in a trap. When the sensor detects a change, the STM32N6 lights the passage, takes a photo and checks what to save. The unit runs on its own after programming. A Windows viewer exports the images from the microSD card.

Start with [How it works](docs/how-it-works.md) for the full flow, the main settings and what affects capture speed.

## Hardware and firmware

| Component | Role |
| --- | --- |
| STM32N6570-DK Discovery | Runs the firmware and holds the camera images in external RAM. The existing CubeIDE project name contains `nucleo`. |
| B-CAMS-IMX / Sony IMX335 | Captures a 2592 × 1944 color image of the passage. |
| VL53L5CX | Measures distance and return signal in 16 zones by default. |
| WS2812 illumination | Lights the passage for the photo. |
| microSD and DS3231 | The card holds the photos. The DS3231 clock supplies the time when set. |

## Capture modes

| Use | Build configuration | Output |
| --- | --- | --- |
| Automatic capture **(default on `feature-cameraToF`)** | `CAPTURE_MODE=4`, `CAM_OVERVIEW_CROP=1` | One 5 MP exposure per accepted ToF event. The image check decides whether to save an overview alone or an overview paired with a crop or a full-size image. Blank and repeat images can be skipped. |
| On-demand capture | `CAPTURE_MODE=0`, `TEST_TOF_MODE=0` | Press USER (PC13) for one 2592 × 1944 full-field record with the default mode-0 settings. Useful for camera, light and card checks. |
| Callback-batch capture | `CAPTURE_MODE=4`, `CAM_OVERVIEW_CROP=0`, `CAM_NATIVE_ROI=1` | Two separately timed 640 × 768 frames of the configured fixed window by default. |
| ToF detection only | `CAPTURE_MODE=4`, `TEST_TOF_MODE=1` | The serial log and board LED show detections. Press USER to learn the empty passage again. |

The [firmware guide](docs/firmware-guide.md#operating-modes) covers the remaining capture variants, task flow and configuration dependencies.

## Build and first capture

1. Check out `feature-cameraToF`. In STM32CubeIDE, choose **File → Import → General → Existing Projects into Workspace** and select the [CubeIDE project directory](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/STM32CubeIDE/NUCLEO-N657X0-Q). Leave **Copy projects into workspace** unchecked so the linked sources resolve in the checkout.
2. Select the profile in [`Inc/app_config.h`](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Inc/app_config.h), build the project and use the included ST-LINK Debug configuration. The profile is compiled into the image. Rebuild and reload it after changing the header.
3. Use a dedicated capture microSD card. The firmware writes images directly to the card. Use the viewer to open or export them.
4. Open the target UART at **115200 baud, 8N1**. In on-demand mode, press USER and look for a saved snapshot. In the default mode, keep the passage empty during ToF baseline learning and through the **first accepted camera exposure**. That image teaches the camera what the empty passage looks like. Look for `empty_reference_saved`, then introduce a test target and look for the camera selection and SD save messages.
5. When writing has completed, stop the firmware and move the card to the host. Scan it and verify the result in the viewer.

## Inspect captures on Windows

From a Python environment on Windows, install the viewer dependencies and run it with administrator access for raw drive access.

```powershell
python -m pip install PySide6 numpy opencv-python
python .\SD_Image_Viewer.py
```

Select the physical microSD, **Refresh**, then **Scan Snapshots**. Inspect images at native scale and use **Download All** or **Extract Selected** to export PNGs. **Format SD** erases the captures found by the scan and resets where the firmware will write the next image. Confirm the selected physical disk before using it.

## Documentation

| Guide | Use |
| --- | --- |
| [Firmware and operating modes](docs/firmware-guide.md) | Follow startup, ToF, camera/ROI and SD from the entry points to the stored records. Includes configuration and bench checks. |
| [Camera pipeline and extension guide](docs/MASSIF-Camera-Pipeline.md) | See how the camera finds objects, makes crops, reuses image buffers and can call an external detector. |
| [Optical analysis](docs/reference/optical-analysis.md) | Compare the 6, 8 and 10 mm f/8 lenses. |
| [Python image filter](docs/reference/python-image-filter.md) | Understand the separate host-side image-filtering experiment. [Spanish reference](docs/reference/python-image-filter-es.md). |
| [Engineering archive](docs/archive/README.md) | Find earlier designs, bench observations, experiments and field notes. |

The [firmware project](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main) contains `Src/`, `Inc/`, `Lib/` and the ST package. The repository root contains the [SD viewer](SD_Image_Viewer.py), the [ToF zone monitor](vl53l5cx_zone_monitor.py) and the [ToF datalogger](vl53l5cx_datalogger.py). The ToF tools read measurements printed over the serial port. Enable `VL53L5CX_DET_DEBUG_ZFRAME` in the firmware for live zone readings.
