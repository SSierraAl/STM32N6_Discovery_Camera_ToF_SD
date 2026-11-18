# MASSIF – Edge Imaging Monitoring System

**Project:** MASSIF – Monitoring Automatisé et Systèmes de Surveillance Intelligents de la biodiversité des insectes dans les écosystèmes Forestiers français

## Overview

This repository focuses on the development of an embedded imaging and monitoring system for the automated observation of insects passing through a dedicated trapping structure.

The development is part of the MASSIF project, which aims to establish a network of autonomous monitoring devices deployed across French forest ecosystems. These devices are intended to support long-term biodiversity monitoring by collecting observations of insects, which can serve as sensitive indicators of environmental and ecological changes.

The main engineering objective is to develop a robust, configurable, and energy-efficient embedded system capable of operating autonomously under outdoor conditions, with minimal human intervention.

The system is built around the **STM32N6 microcontroller family**, with firmware designed for the STM32N6570-DK Discovery board and potential adaptation to more compact embedded platforms, including the OhmLab Neuro N6.

### System architecture and operation

The acquisition system observes a constrained passage region through which insects are guided by the mechanical design of the trap.

A multizone Time-of-Flight (ToF) sensor continuously monitors this region, using adaptive reference measurements and configurable detection algorithms to identify the presence or movement of an object. When a detection satisfies the configured triggering conditions, the system activates the illumination and initiates the camera acquisition sequence.

The firmware then analyzes the captured image against an empty-passage reference to determine the most appropriate output. Depending on the detected content and the selected configuration, the system can:

- Store an overview image of the monitored passage.
- Extract and store a region of interest (ROI) containing the detected object.
- Preserve the full-resolution image when cropping is not appropriate.
- Reject empty images and suppress certain repeated detections.

This approach is intended to reduce unnecessary data storage while preserving useful visual information for subsequent analysis.

Images are written directly to a microSD card using a dedicated raw storage format. A companion Python-based Windows application provides image inspection, reconstruction, and export to standard PNG files.

The firmware also provides multiple acquisition modes and configurable detection parameters, allowing the system to be used for field monitoring, laboratory experiments, sensor calibration, and image dataset collection.

## Hardware and firmware

| Component | Role |
| --- | --- |
| STM32N6570-DK Discovery / OhmLab Neuro N6 | STM32N6-based development and embedded platforms. |
| B-CAMS-IMX / Sony IMX335 | Camera module, 2592 × 1944 pixels (5 MP) color images. |
| VL53L5CX | Multizone Time-of-Flight sensor used for distance measurements and object detection. |
| WS2812B illumination | Ring LED illumination activated during image acquisition. |
| microSD card | Stores captured images using the dedicated raw format. |
| DS3231 RTC | Provides timestamps for the captured images. |

## Capture modes

To support the different stages of development, testing, and deployment, the firmware provides several operating modes selected through compile-time configuration.

| Mode | Build configuration | Output |
| --- | --- | --- |
| **Automatic capture** | `CAPTURE_MODE=4`, `CAM_OVERVIEW_CROP=1` | One 5 MP camera exposure per accepted ToF event. Image analysis determines whether to save an overview alone, an overview with a cropped image, or a full-resolution image. The system also permit downsample the final image resolution. |
| **On-demand capture** | `CAPTURE_MODE=0`, `TEST_TOF_MODE=0` | Captures one full-field 2592 × 1944 image per press of the USER button (PC13), using the default mode-0 configuration. Particularly useful for controlled acquisition and dataset generation. |
| **ToF detection only** | `CAPTURE_MODE=4`, `TEST_TOF_MODE=1` | Runs the ToF detection pipeline without image acquisition, allowing calibration, and tuning of detection sensitivity, adaptive thresholds, and triggering behavior. |

Additional acquisition variants and configuration dependencies are described in the [Firmware and Operating Modes Guide](docs/firmware-guide.md#operating-modes).

## Image viewer

A dedicated Python-based image viewer is included to inspect and reconstruct the images stored on the microSD card.

Since the embedded system writes images in a custom raw format, the viewer provides the tools required to locate stored captures, reconstruct their image data, inspect the results, and export them as PNG files. For it is used it is necessary run the terminal with administrator rights.

### Installation and execution

From a Python environment on Windows, install the dependencies:

```powershell
python -m pip install PySide6 numpy opencv-python
```

Run the application with administrator privileges to allow raw access to the SD card:

```powershell
python .\SD_Image_Viewer.py
```

### Basic workflow

1. **Refresh:** Detect the available storage devices connected to the computer.
2. **Scan Snapshots:** Search the selected device for images stored by the embedded system.
3. **Inspect:** Browse the captured images, their resolutions, categories, timestamps, and filenames.
4. **Download All / Extract Selected:** Reconstruct and export all images or a selected subset as PNG files.
5. **Format SD:** Clear the capture storage and reset the internal write position and counters, allowing a new acquisition sequence to begin.

**Warning:** The Format SD operation removes stored capture data. Export any images you want to preserve before using it.

## Documentation

The following documents provide a more detailed explanation of the system architecture, acquisition algorithms, firmware configuration, and supporting tools.

| Document | Scope |
| --- | --- |
| [System operation and performance](docs/how-it-works.md) | Detection and capture sequence, image selection, nominal settings, and performance metrics. |
| [Firmware and operating modes](docs/firmware-guide.md) | Firmware entry points, operating modes, configuration parameters, and bench validation. |
| [ToF detection and spatial analysis](docs/tof-detection.md) | Zone baselines, spatial residuals, trigger paths, and baseline updates. |
| [Camera pipeline and extension guide](docs/MASSIF-Camera-Pipeline.md) | Image analysis, crop geometry, buffer reuse, and external detector interface. |
| [SD storage and image reconstruction](docs/sd-storage-and-reconstruction.md) | Sector layout, persistent write position, viewer scanning, and PNG export. |
| [Optical analysis for lens selection](docs/reference/optical-analysis.md) | Systematic comparison of 6, 8, and 10 mm f/8 lenses. |
| [Python image filter for CV cropping](docs/reference/python-image-filter.md) | Host-side image filtering experiments and ROI analysis. |

### Repository organization

The [firmware project](x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main) contains the main embedded implementation, including the `Src/`, `Inc/`, and `Lib/` directories, along with the associated ST software package and development environment.

The repository root also provides supporting Python tools:

- [SD Image Viewer](SD_Image_Viewer.py): Image reconstruction, visualization, and export from the microSD card.
- [ToF Zone Monitor](vl53l5cx_zone_monitor.py): Live visualization of multizone ToF measurements received over the serial interface.
- [ToF Datalogger](vl53l5cx_datalogger.py): Recording and analysis of ToF measurements for debugging and calibration.

For live zone readings, enable `VL53L5CX_DET_DEBUG_ZFRAME` in the firmware configuration.

## Archive of previous documentation and interations

Earlier firmware architectures, experimental approaches, calibration procedures, and technical investigations carried out during development. These documents provide additional context on the design decisions and evolution of the system. Some describe earlier firmware configurations or hardware arrangements and should therefore be considered references rather than the current implementation.

| Topic | Historical records |
| --- | --- |
| Architecture and task design | [System architecture](docs/archive/project/MASSIF-Camera-System-Architecture.md), [thread architecture](docs/archive/firmware/07_THREAD_ARCHITECTURE.md), [original project walkthrough](docs/archive/firmware-package/PROJECT_DOCUMENTATION.md) |
| ToF and field calibration | [Single-sensor field guide](docs/archive/project/TOF_DETECTION_FIELD_GUIDE.md), [noise-rejection proposal](docs/archive/project/tof_detection_noise_mad_strategy.md), [Spanish walkthrough](docs/archive/firmware/TOF_ALGORITHM_EXPLAINED_ES.md) |
| Acquisition and ROI development | [Multi-resolution strategy](docs/archive/project/MULTI_RES_STRATEGY.md), [capture mode history](docs/archive/firmware/CAPTURE_MODES_REFERENCE.md), [callback-batch notes](docs/archive/firmware/CALLBACK_BATCH_MODE_GUIDE.md), [batch coexistence test plan](docs/archive/firmware-package/BATCH_MODE_TEST_PLAN.md) |
| Camera quality and timing | [Image-quality review](docs/archive/firmware/CAMERA_IMAGE_QUALITY_REVIEW.md), [exposure tests](docs/archive/firmware/EXPOSURE_AND_TIMING_VALIDATION.md), [binning exploration](docs/archive/firmware/IMX335_BINNING_EXPLORATION_PLAN.md), [low-power study](docs/archive/firmware/LOW_POWER_MODE4_ANALYSIS.md) |
| Storage and diagnostics | [SD architecture](docs/archive/firmware/03_SD_Storage_Architecture.md), [SD performance study](docs/archive/firmware-package/SD_CARD_ANALYSIS_AND_CAPTURE_MODES.md), [performance diagnostics](docs/archive/firmware/PERFORMANCE_DEBUG_GUIDE.md) |
| Earlier configuration and terminology | [Field parameters](docs/archive/firmware/09_TUNING_PARAMETERS.md), [tuning guide](docs/archive/firmware/TUNING_GUIDE.md), [terminology glossary](docs/archive/firmware/PROTOCOLS_AND_ABBREVIATIONS.md) |