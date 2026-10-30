// 1. ADD THESE TWO HEADERS
#include <algorithm>
#include <type_traits>
#include <math.h>

#include "aruco_detect.h"
#include "ArucoLite.h"
#include <string.h>

#define ARUCO_DB ARUCO_DB_4X4_1000
#include "ArucoLite.h"

#define FOCAL_LENGTH_PX     277.0f

typedef ArucoLite<320, 240, 16, false> MyAruco_t;
static MyAruco_t aruco;
static MyAruco_t::Frame aruco_frame;

static void rgb565_to_gray(camera_fb_t *fb, uint8_t *gray_buf) {
    uint8_t *buf = (uint8_t *)fb->buf;
    int num_pixels = fb->width * fb->height;
    for (int i = 0; i < num_pixels; i++) {
        uint16_t px = (buf[i * 2] << 8) | buf[i * 2 + 1];
        uint8_t r = (px >> 11) & 0x1F;
        uint8_t g = (px >> 5)  & 0x3F;
        uint8_t b =  px        & 0x1F;
        gray_buf[i] = (uint8_t)(((r * 77) + (g * 150) + (b * 29)) >> 8);
    }
}

int aruco_detect_frame(camera_fb_t *fb, float marker_size_mm,
                       aruco_result_t *results, int result_capacity) {
    if (fb == nullptr || results == nullptr || result_capacity <= 0 ||
        fb->width != 320 || fb->height != 240 || marker_size_mm <= 0.0f) {
        return 0;
    }

    // 3. FIX POINTER: Cast the 2D array to a 1D pointer
    rgb565_to_gray(fb, (uint8_t*)aruco_frame.data);

    aruco.process(aruco_frame);

    int count = aruco.arucos_found;
    if (count > result_capacity) count = result_capacity;
    for (int i = 0; i < count; ++i) {
        const auto &marker = aruco.result[i];
        aruco_result_t &result = results[i];
        result = {};
        result.detected = true;
        result.marker_id = (int)marker.aruco_idx;
        float sx = 0, sy = 0;
        for (int p = 0; p < 4; p++) {
            sx += marker.pt[p].x;
            sy += marker.pt[p].y;
        }
        result.cx = sx / 4.0;
        result.cy = sy / 4.0;

        float top_w = hypot(marker.pt[1].x - marker.pt[0].x, marker.pt[1].y - marker.pt[0].y);
        float bot_w = hypot(marker.pt[2].x - marker.pt[3].x, marker.pt[2].y - marker.pt[3].y);
        result.width_px = (top_w + bot_w) / 2.0;

        if (result.width_px > 0) {
            result.distance_mm = (marker_size_mm * FOCAL_LENGTH_PX) / result.width_px;
        }
    }

    return count;
}
