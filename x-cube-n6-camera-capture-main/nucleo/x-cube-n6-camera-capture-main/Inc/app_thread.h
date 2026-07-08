/**
 * ******************************************************************************
 * @file    app_thread.h
 * @brief   Multi-threaded architecture for Camera + Sensor + Storage
 *
 *   Thread Model:
 *     - SENSOR_TASK  (priority IDLE+2): VL53L5CX ToF monitoring
 *     - CAMERA_TASK  (priority IDLE+3): Camera acquisition (continuous/snapshot)
 *     - STORAGE_TASK (priority IDLE+1): SD card image storage
 *
 *   IPC:
 *     - camera_cmd_queue:    Sensor/Ctrl -> Camera commands
 *     - storage_cmd_queue:   Camera      -> Storage commands
 *     - sensor_event_queue:  Camera/Storage -> Sensor notifications
 *     - camera_ready_sem:    Camera signals "snapshot captured"
 *     - storage_done_sem:    Storage signals "SD write complete"
 * ******************************************************************************
 */

#ifndef APP_THREAD_H
#define APP_THREAD_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ================================================================
   SENSOR STATES
   ================================================================ */

typedef enum {
    SENSOR_STATE_IDLE = 0,
    SENSOR_STATE_RUNNING,       /* Normal ToF monitoring */
    SENSOR_STATE_PAUSED,        /* Paused during camera+SD operations */
    SENSOR_STATE_STOPPED        /* Fully stopped */
} SensorState_t;

/* ================================================================
   CAMERA COMMANDS (Sensor/Ctrl -> Camera)
   ================================================================ */

typedef enum {
    CAM_CMD_NOOP = 0,
    CAM_CMD_SNAP,               /* Take a snapshot (continuous mode) */
    CAM_CMD_START_CONTINUOUS,   /* Start continuous capture */
    CAM_CMD_STOP_CONTINUOUS,    /* Stop continuous capture */
    CAM_CMD_CAPTURE_SINGLE,     /* Full init->capture->deinit cycle */
    CAM_CMD_GET_STATUS          /* Request status */
} CameraCmdType_t;

typedef struct {
    CameraCmdType_t type;
    uint8_t         *dest_buf;   /* Output buffer for snap (optional) */
    uint32_t        dest_size;   /* Buffer size */
    uint32_t        width;
    uint32_t        height;
    uint32_t        fps;
    uint32_t        warmup_frames;
} CameraCmd_t;

/* ================================================================
   CAMERA EVENTS (Camera -> Sensor)
   ================================================================ */

typedef enum {
    CAM_EVENT_NONE = 0,
    CAM_EVENT_SNAP_READY,       /* Snapshot captured, frame available */
    CAM_EVENT_SNAP_FAILED,      /* Snapshot failed */
    CAM_EVENT_CONTINUOUS_STARTED,
    CAM_EVENT_CONTINUOUS_STOPPED,
    CAM_EVENT_ERROR
} CameraEvent_t;

typedef struct {
    CameraEvent_t type;
    int           error_code;
    uint32_t      elapsed_ms;
} CameraEvent_typed;

/* ================================================================
   STORAGE COMMANDS (Camera -> Storage)
   ================================================================ */

typedef enum {
    STORAGE_CMD_NOOP = 0,
    STORAGE_CMD_SAVE,           /* Save image to SD card */
    STORAGE_CMD_GET_STATUS,
    STORAGE_CMD_GET_FREE_SPACE
} StorageCmdType_t;

typedef struct {
    StorageCmdType_t type;
    const uint8_t    *image_buf;
    uint32_t         image_size;
    uint32_t         width;
    uint32_t         height;
    uint32_t         pixel_format;
    uint32_t         snap_id;
} StorageCmd_t;

/* ================================================================
   STORAGE EVENTS (Storage -> Camera/Sensor)
   ================================================================ */

typedef enum {
    STORAGE_EVENT_NONE = 0,
    STORAGE_EVENT_SAVE_DONE,    /* SD write complete */
    STORAGE_EVENT_SAVE_FAILED,  /* SD write failed */
    STORAGE_EVENT_OVERFLOW      /* SD card full */
} StorageEventType_t;

typedef struct {
    StorageEventType_t type;
    uint32_t           block_used;
    uint32_t           elapsed_ms;
    int                error_code;
} StorageEvent_t;

/* ================================================================
   THREAD STACK SIZES
   ================================================================ */

#define SENSOR_TASK_STACK_SIZE    (4 * configMINIMAL_STACK_SIZE)
#define CAMERA_TASK_STACK_SIZE    (4 * configMINIMAL_STACK_SIZE)
#define STORAGE_TASK_STACK_SIZE   (6 * configMINIMAL_STACK_SIZE)

/* ================================================================
   QUEUE SIZES
   ================================================================ */

#define CAMERA_CMD_QUEUE_LEN      4
#define STORAGE_CMD_QUEUE_LEN     4
#define SENSOR_EVENT_QUEUE_LEN    4

/* ================================================================
   PUBLIC: External Handles (created by main, used by tasks)
   ================================================================ */

/* IPC Queues */
extern QueueHandle_t camera_cmd_queue;
extern QueueHandle_t storage_cmd_queue;
extern QueueHandle_t sensor_event_queue;

/* Semaphores */
extern SemaphoreHandle_t camera_ready_sem;
extern SemaphoreHandle_t storage_done_sem;

/* Shared state */
extern volatile SensorState_t g_sensor_state;

/* ================================================================
   INITIALIZATION
   ================================================================ */

/**
 * @brief Create all IPC objects (queues, semaphores)
 * @return pdTRUE on success, pdFALSE on failure
 */
BaseType_t IPC_Init(void);

/**
 * @brief Delete all IPC objects (cleanup)
 */
void IPC_Deinit(void);

/* ================================================================
   TASK ENTRY POINTS
   ================================================================ */

/**
 * @brief Sensor monitoring task
 *   - Continuous VL53L5CX_Update() loop
 *   - On detection: pause self, request camera snap, wait for completion, resume
 */
void sensor_task(void *arg);

/**
 * @brief Camera acquisition task
 *   - Receives commands from camera_cmd_queue
 *   - Handles continuous mode, snapshots, single captures
 *   - Signals camera_ready_sem when snap done
 *   - Forwards frame to storage via storage_cmd_queue
 */
void camera_task(void *arg);

/**
 * @brief Storage task
 *   - Receives save commands from storage_cmd_queue
 *   - Performs SD card writes
 *   - Signals storage_done_sem when complete
 */
void storage_task(void *arg);

/* ================================================================
   HIGH-LEVEL API (for triggering captures from anywhere)
   ================================================================ */

/**
 * @brief Request a full capture cycle: snap + save
 *   This is the main entry point when an insect is detected.
 *   It blocks until the entire cycle (sensor pause -> snap -> save -> resume) is done.
 *
 * @param timeout_ms  Max time to wait for completion (0 = forever)
 * @return 0 on success, -1 on timeout, -2 on queue error
 */
int Capture_RequestSnapshot(uint32_t timeout_ms);

/**
 * @brief Request a snapshot only (non-blocking)
 * @return pdTRUE on success
 */
BaseType_t Capture_RequestSnapOnly(void);

/**
 * @brief Check if a capture is currently in progress
 * @return 1 if busy, 0 if idle
 */
int Capture_IsBusy(void);

#endif /* APP_THREAD_H */