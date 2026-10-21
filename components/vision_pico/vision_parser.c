#include <float.h>
#include <limits.h>
#include <stddef.h>
#include "vision_internal.h"

typedef struct { const char *next, *end; } Cursor;

void vision_clear_result(VisionResult *result, VisionStatus status)
{
    *result = (VisionResult){
        .status = status, .kind = VISION_KIND_NONE,
        .colour = VISION_COLOUR_NONE, .marker_id = -1, .x_px = -1, .y_px = -1
    };
}

static int hex_digit(char ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

static bool comma(Cursor *cursor)
{
    if (cursor->next == cursor->end || *cursor->next != ',') return false;
    ++cursor->next;
    return true;
}

static bool unsigned_integer(Cursor *cursor, int32_t *value)
{
    const char *first = cursor->next;
    int32_t parsed = 0;
    while (cursor->next != cursor->end &&
           *cursor->next >= '0' && *cursor->next <= '9') {
        int digit = *cursor->next++ - '0';
        if (parsed > (INT32_MAX - digit) / 10) return false;
        parsed = parsed * 10 + digit;
    }
    *value = parsed;
    return cursor->next != first;
}

/* ESP32 sends positive fixed-point %.1f values, not exponents/NaN/Inf. */
static bool positive_decimal(Cursor *cursor, float *value)
{
    const char *first = cursor->next;
    float parsed = 0.0f;
    while (cursor->next != cursor->end &&
           *cursor->next >= '0' && *cursor->next <= '9') {
        int digit = *cursor->next++ - '0';
        if (parsed >= FLT_MAX / 10.0f - 1.0f) return false;
        parsed = parsed * 10.0f + (float)digit;
    }
    if (cursor->next == first) return false;
    if (cursor->next != cursor->end && *cursor->next == '.') {
        first = ++cursor->next;
        float place = 0.1f;
        while (cursor->next != cursor->end &&
               *cursor->next >= '0' && *cursor->next <= '9') {
            parsed += (float)(*cursor->next++ - '0') * place;
            place *= 0.1f;
        }
        if (cursor->next == first) return false;
    }
    *value = parsed;
    return parsed > 0.0f && parsed <= FLT_MAX;
}

bool vision_parse_line(const char *line, unsigned length, VisionResult *result)
{
    if (result == NULL) return false;
    vision_clear_result(result, VISION_INVALID_PACKET);
    if (line == NULL || length < 2 ||
        (line[0] != 'A' && line[0] != 'C') || line[1] != ',') return false;
    if (length >= VISION_RX_LINE_SIZE) return true;
    if (length && line[length - 1] == '\n') --length;
    if (length && line[length - 1] == '\r') --length;

    unsigned payload_length = 0;
    uint8_t checksum = 0;
    while (payload_length < length && line[payload_length] != '*') {
        checksum ^= (uint8_t)line[payload_length++];
    }
    if (payload_length + 3 != length) return true;
    int hi = hex_digit(line[payload_length + 1]);
    int lo = hex_digit(line[payload_length + 2]);
    if (hi < 0 || lo < 0 || checksum != (uint8_t)((hi << 4) | lo)) return true;

    VisionResult parsed;
    vision_clear_result(&parsed, VISION_OK);
    Cursor cursor = { line + 2, line + payload_length };
    if (line[0] == 'C') {
        parsed.kind = VISION_COLOUR;
        if (cursor.next == cursor.end) return true;
        char colour = *cursor.next++;
        if (colour == 'N') {
            if (cursor.next != cursor.end) return true;
            parsed.status = VISION_NO_OBJECT;
            *result = parsed;
            return true;
        }
        switch (colour) {
        case 'R': parsed.colour = VISION_RED; break;
        case 'G': parsed.colour = VISION_GREEN; break;
        case 'B': parsed.colour = VISION_BLUE; break;
        default: return true;
        }
        if (!comma(&cursor)) return true;
    } else {
        parsed.kind = VISION_ARUCO;
        if (!unsigned_integer(&cursor, &parsed.marker_id) || !comma(&cursor)) return true;
    }

    int32_t x, y;
    if (!unsigned_integer(&cursor, &x) || x >= VISION_IMAGE_WIDTH_PX ||
        !comma(&cursor) || !unsigned_integer(&cursor, &y) ||
        y >= VISION_IMAGE_HEIGHT_PX) return true;
    if (parsed.kind == VISION_ARUCO &&
        (!comma(&cursor) || !positive_decimal(&cursor, &parsed.marker_width_px) ||
         !comma(&cursor) || !positive_decimal(&cursor, &parsed.distance_mm))) return true;
    if (cursor.next != cursor.end) return true;
    parsed.x_px = (int16_t)x;
    parsed.y_px = (int16_t)y;
    *result = parsed;
    return true;
}
