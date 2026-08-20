# Insect Capture System Documentation

This library presents the STM32N6 insect-imaging firmware at the level needed for a project review, technical handover, or future development. It describes the implementation currently in this repository; configuration values should always be checked in [`Inc/app_config.h`](../Inc/app_config.h).

## Start here

| Audience | Read |
| --- | --- |
| Project sponsors and reviewers | [Executive summary](01-executive-summary.md) |
| System architects and embedded developers | [System architecture](02-system-architecture.md) |
| Operators and experiment owners | [Operating modes](03-operating-modes.md) |
| Developers changing behavior or performance | [Configuration guide](04-configuration-guide.md) |
| Developers debugging timing or concurrency | [Runtime and threads](05-runtime-and-threads.md) |
| Test and field teams | [Operating procedure](06-operating-procedure.md) |

## Scope and terminology

* **Snapshot** is one raw YUV422 image captured from the IMX335.
* **Event** is one manual button press or one validated ToF insect detection.
* **Burst** is the group of images captured for one detection event.
* The firmware stores images by direct SD-card block I/O in the active standalone capture path. This is not a FAT/Folder-based workflow.

## Authoritative sources

The documents summarize source behavior, not a separate specification. Key implementation files are `Inc/app_config.h`, `Src/main.c`, `Src/app_thread.c`, `Src/app_cam.c`, `Src/vl53l5cx_detection.c`, and `Src/ws2812.c`.
