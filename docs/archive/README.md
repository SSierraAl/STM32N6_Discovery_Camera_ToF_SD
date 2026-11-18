# Engineering archive

These documents record earlier firmware layouts, field settings, experiments and design proposals. Consult the [firmware guide](../firmware-guide.md) and the linked source for the current behavior of `feature-cameraToF`. Historical measurements belong to the board and settings used at the time.

| Topic | Historical records |
| --- | --- |
| Architecture and task design | [System architecture](project/MASSIF-Camera-System-Architecture.md), [thread architecture](firmware/07_THREAD_ARCHITECTURE.md), [original project walkthrough](firmware-package/PROJECT_DOCUMENTATION.md) |
| ToF and field calibration | [Single-sensor field guide](project/TOF_DETECTION_FIELD_GUIDE.md), [noise-rejection proposal](project/tof_detection_noise_mad_strategy.md), [Spanish walkthrough](firmware/TOF_ALGORITHM_EXPLAINED_ES.md) |
| Acquisition and ROI development | [Multi-resolution strategy](project/MULTI_RES_STRATEGY.md), [capture mode history](firmware/CAPTURE_MODES_REFERENCE.md), [callback-batch notes](firmware/CALLBACK_BATCH_MODE_GUIDE.md), [batch coexistence test plan](firmware-package/BATCH_MODE_TEST_PLAN.md) |
| Camera quality and timing | [Image-quality review](firmware/CAMERA_IMAGE_QUALITY_REVIEW.md), [exposure tests](firmware/EXPOSURE_AND_TIMING_VALIDATION.md), [binning exploration](firmware/IMX335_BINNING_EXPLORATION_PLAN.md), [low-power study](firmware/LOW_POWER_MODE4_ANALYSIS.md) |
| Storage and diagnostics | [SD architecture](firmware/03_SD_Storage_Architecture.md), [SD performance study](firmware-package/SD_CARD_ANALYSIS_AND_CAPTURE_MODES.md), [performance diagnostics](firmware/PERFORMANCE_DEBUG_GUIDE.md) |
| Earlier configuration and terminology | [Field parameters](firmware/09_TUNING_PARAMETERS.md), [tuning guide](firmware/TUNING_GUIDE.md), [terminology glossary](firmware/PROTOCOLS_AND_ABBREVIATIONS.md) |

The folders retain the earlier project, firmware package and firmware project notes. Identical copies of six firmware guides were consolidated here. The deduplicated [optical analysis](../reference/optical-analysis.md) and the [Python filtering notes](../reference/python-image-filter.md) are under `docs/reference/`.
