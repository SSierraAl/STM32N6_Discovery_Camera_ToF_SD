# MASSIF insect imaging firmware

The MASSIF imaging system records insect passage through a trap. A multizone time-of-flight (ToF) sensor triggers an illuminated capture on the STM32N6. The firmware compares the captured image with an empty-passage reference to determine whether to store an overview alone or pair it with a crop or a full-resolution image. Blank and repeat detections can be discarded. Images are stored on microSD and exported with a Windows viewer.

## Hardware and firmware

| Component | Role |
| --- | --- |
| STM32N6570-DK Discovery | Runs the firmware and holds the camera images in external RAM. The existing CubeIDE project name contains `nucleo`. |
| B-CAMS-IMX / Sony IMX335 | Captures a 2592 × 1944 color image of the passage. |
| VL53L5CX | Measures distance and return signal in 16 zones by default. |
| WS2812 illumination | Lights the passage for the photo. |
| microSD and DS3231 | The card holds the photos. The DS3231 clock supplies the time when set. |

## Capture modes

| Mode | Build configuration | Output |
| --- | --- | --- |
| Automatic capture **(default on `feature-cameraToF`)** | `CAPTURE_MODE=4`, `CAM_OVERVIEW_CROP=1` | One 5 MP exposure per accepted ToF event. The image check decides whether to save an overview alone or an overview paired with a crop or a full-size image. Blank and repeat images can be skipped. |
| On-demand capture | `CAPTURE_MODE=0`, `TEST_TOF_MODE=0` | One 2592 × 1944 full-field image per press of USER (PC13) with the default mode-0 settings. |
| Callback-batch capture | `CAPTURE_MODE=4`, `CAM_OVERVIEW_CROP=0`, `CAM_NATIVE_ROI=1` | Two separately timed 640 × 768 frames of the configured fixed window by default. |
| ToF detection only | `CAPTURE_MODE=4`, `TEST_TOF_MODE=1` | The serial log and board LED show detections. USER initiates a new ToF baseline measurement. |

The [firmware guide](docs/firmware-guide.md#operating-modes) covers the remaining capture variants, task flow and configuration dependencies.

## Build and initial validation

1. Check out `feature-cameraToF`. In STM32CubeIDE, choose **File → Import → General → Existing Projects into Workspace** and select the [CubeIDE project directory](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/STM32CubeIDE/NUCLEO-N657X0-Q). Leave **Copy projects into workspace** unchecked so the linked sources resolve in the checkout.
2. Select the profile in [`Inc/app_config.h`](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Inc/app_config.h), build the project and use the included ST-LINK Debug configuration. The profile is compiled into the image. Rebuild and reload it after changing the header.
3. Use a dedicated capture microSD card. The firmware writes images directly to the card. Use the viewer to open or export them.
4. Open the target UART at **115200 baud, 8N1**. In automatic mode, an empty passage is required during ToF baseline acquisition and during the first camera exposure used to establish the image reference.
5. Complete the applicable bench test below. After storage completes, stop the firmware and scan the card in the viewer.

| Test condition | Expected observation |
| --- | --- |
| USER press in on-demand mode | One full-field image on the card. |
| First qualifying exposure with the passage empty in automatic mode | The camera reports `empty_reference_saved` and creates no image record. |
| A test target crossing the passage after reference acquisition | The camera reports its selection in `[CAM][SELECT]`. When the selected action calls for storage, the corresponding image records are present on the card. |

## Inspect captures on Windows

From a Python environment on Windows, install the viewer dependencies and run it with administrator access for raw drive access.

```powershell
python -m pip install PySide6 numpy opencv-python
python .\SD_Image_Viewer.py
```

Select the physical microSD, **Refresh**, then **Scan Snapshots**. Inspect images at native scale and use **Download All** or **Extract Selected** to export PNGs. **Format SD** erases the captures found by the scan and resets where the firmware will write the next image. Confirm the selected physical disk before using it.

## Documentation

| Document | Scope |
| --- | --- |
| [System operation and performance](docs/how-it-works.md) | Detection and capture sequence, image selection, nominal settings and performance metrics. |
| [Firmware and operating modes](docs/firmware-guide.md) | Firmware entry points, operating modes, configuration and bench validation. |
| [Camera pipeline and extension guide](docs/MASSIF-Camera-Pipeline.md) | Image analysis, crop geometry, buffer reuse and the external detector interface. |
| [Optical analysis](docs/reference/optical-analysis.md) | Comparison of 6, 8 and 10 mm f/8 lenses. |
| [Python image filter](docs/reference/python-image-filter.md) | Host-side image filtering experiment. [Spanish reference](docs/reference/python-image-filter-es.md). |
| [Engineering archive](docs/archive/README.md) | Earlier designs, bench observations, experiments and field notes. |

The [firmware project](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main) contains `Src/`, `Inc/`, `Lib/` and the ST package. The repository root contains the [SD viewer](SD_Image_Viewer.py), the [ToF zone monitor](vl53l5cx_zone_monitor.py) and the [ToF datalogger](vl53l5cx_datalogger.py). The ToF tools read measurements printed over the serial port. Enable `VL53L5CX_DET_DEBUG_ZFRAME` in the firmware for live zone readings.
