#pragma once
#include "esp_camera.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool detected;
    int marker_id;
    float cx, cy;       // Centroid in pixels
    float width_px;     // Apparent width for distance estimation
    float distance_mm;
} aruco_result_t;

aruco_result_t aruco_detect_frame(camera_fb_t *fb);

#ifdef __cplusplus
}
#endif