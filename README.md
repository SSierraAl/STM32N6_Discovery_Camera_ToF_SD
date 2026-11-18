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
| [ToF detection and spatial analysis](docs/tof-detection.md) | Zone baselines, spatial residuals, trigger paths and baseline updates. |
| [Camera pipeline and extension guide](docs/MASSIF-Camera-Pipeline.md) | Image analysis, crop geometry, buffer reuse and the external detector interface. |
| [SD storage and image reconstruction](docs/sd-storage-and-reconstruction.md) | Sector layout, persistent write position, viewer scanning and PNG export. |
| [Optical analysis](docs/reference/optical-analysis.md) | Comparison of 6, 8 and 10 mm f/8 lenses. |
| [Python image filter](docs/reference/python-image-filter.md) | Host-side image filtering experiment. [Spanish reference](docs/reference/python-image-filter-es.md). |

The [firmware project](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main) contains `Src/`, `Inc/`, `Lib/` and the ST package. The repository root contains the [SD viewer](SD_Image_Viewer.py), the [ToF zone monitor](vl53l5cx_zone_monitor.py) and the [ToF datalogger](vl53l5cx_datalogger.py). The ToF tools read measurements printed over the serial port. Enable `VL53L5CX_DET_DEBUG_ZFRAME` in the firmware for live zone readings.

## Engineering archive

The archive documents earlier firmware designs, field settings and experiments. Measurements refer to the hardware and configuration used in each study.

| Topic | Historical records |
| --- | --- |
| Architecture and task design | [System architecture](docs/archive/project/MASSIF-Camera-System-Architecture.md), [thread architecture](docs/archive/firmware/07_THREAD_ARCHITECTURE.md), [original project walkthrough](docs/archive/firmware-package/PROJECT_DOCUMENTATION.md) |
| ToF and field calibration | [Single-sensor field guide](docs/archive/project/TOF_DETECTION_FIELD_GUIDE.md), [noise-rejection proposal](docs/archive/project/tof_detection_noise_mad_strategy.md), [Spanish walkthrough](docs/archive/firmware/TOF_ALGORITHM_EXPLAINED_ES.md) |
| Acquisition and ROI development | [Multi-resolution strategy](docs/archive/project/MULTI_RES_STRATEGY.md), [capture mode history](docs/archive/firmware/CAPTURE_MODES_REFERENCE.md), [callback-batch notes](docs/archive/firmware/CALLBACK_BATCH_MODE_GUIDE.md), [batch coexistence test plan](docs/archive/firmware-package/BATCH_MODE_TEST_PLAN.md) |
| Camera quality and timing | [Image-quality review](docs/archive/firmware/CAMERA_IMAGE_QUALITY_REVIEW.md), [exposure tests](docs/archive/firmware/EXPOSURE_AND_TIMING_VALIDATION.md), [binning exploration](docs/archive/firmware/IMX335_BINNING_EXPLORATION_PLAN.md), [low-power study](docs/archive/firmware/LOW_POWER_MODE4_ANALYSIS.md) |
| Storage and diagnostics | [SD architecture](docs/archive/firmware/03_SD_Storage_Architecture.md), [SD performance study](docs/archive/firmware-package/SD_CARD_ANALYSIS_AND_CAPTURE_MODES.md), [performance diagnostics](docs/archive/firmware/PERFORMANCE_DEBUG_GUIDE.md) |
| Earlier configuration and terminology | [Field parameters](docs/archive/firmware/09_TUNING_PARAMETERS.md), [tuning guide](docs/archive/firmware/TUNING_GUIDE.md), [terminology glossary](docs/archive/firmware/PROTOCOLS_AND_ABBREVIATIONS.md) |
