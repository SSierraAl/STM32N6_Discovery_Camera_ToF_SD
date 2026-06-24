/**
 ******************************************************************************
 * @file    app.h
 * @author  GPM Application Team (adapted for standalone snapshot mode)
 *
 * @brief   Application entry point and frame callback API.
 *
 *   In the original UVC design, app_run() created three FreeRTOS tasks
 *   (uvc, isp, capture) for USB video streaming. In standalone mode,
 *   app_run() is NOT called — the button thread captures frames directly
 *   via CAM_CaptureSingleFrame().
 *
 *   The frame callback API (APP_SetFrameCallback) is retained for backward
 *   compatibility but is not used in standalone mode.
 ******************************************************************************
 */
#ifndef APP_H
#define APP_H

#include <stdint.h>

/* ================================================================
   LEGACY API (retained for build compatibility, not used in standalone)
   ================================================================ */

/**
 * @brief  Start the UVC camera streaming application.
 *
 *   In standalone mode this function is intentionally NOT called.
 *   When called, it creates the uvc/isp/capture FreeRTOS tasks,
 *   initializes the USB device, and starts the camera pipeline.
 */
void app_run(void);

/**
 * @brief  Frame callback signature.
 *
 *   Invoked by the UVC thread when a frame is ready. In standalone
 *   mode the button thread captures frames directly without using
 *   this callback mechanism.
 *
 * @param  data          Pointer to frame pixel data
 * @param  len           Frame data length in bytes
 * @param  w             Frame width in pixels
 * @param  h             Frame height in pixels
 * @param  payload_type  Frame format identifier (UVCL_PAYLOAD_*)
 */
typedef void (*frame_cb_t)(uint8_t *data, int len, int w, int h, int payload_type);

/**
 * @brief  Register a callback for the next available frame.
 *
 * @param  cb           Callback function pointer
 * @param  skip_frames  Number of warmup frames to skip before invoking cb
 */
void APP_SetFrameCallback(frame_cb_t cb, int skip_frames);

#endif /* APP_H */