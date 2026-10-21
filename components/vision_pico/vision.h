#ifndef VISION_H
#define VISION_H

#include <stdint.h>

/* ESP32 FRAMESIZE_QVGA: image origin top-left, +x right, +y down. */
#define VISION_IMAGE_WIDTH_PX  320
#define VISION_IMAGE_HEIGHT_PX 240

typedef enum {
    VISION_OK,
    VISION_NO_OBJECT,
    VISION_INVALID_PACKET,
    VISION_TIMEOUT
} VisionStatus;

typedef enum { VISION_KIND_NONE, VISION_COLOUR, VISION_ARUCO } VisionKind;
typedef enum {
    VISION_COLOUR_NONE, VISION_RED, VISION_GREEN, VISION_BLUE
} VisionColour;

typedef struct {
    VisionStatus status;
    VisionKind kind;
    VisionColour colour;       /* COLOUR + OK only; NONE otherwise. */
    int32_t marker_id;          /* ARUCO + OK only; -1 otherwise. */
    int16_t x_px, y_px;         /* OK only; -1 otherwise. NOT robot coordinates. */
    float marker_width_px;     /* ARUCO + OK only; 0 otherwise. */
    float distance_mm;         /* ARUCO + OK only: ESP32's estimate, not arm XYZ. */
} VisionResult;

#endif
