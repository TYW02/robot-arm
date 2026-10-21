#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../vision_internal.h"

static VisionResult parse_payload(const char *payload, const char *ending)
{
    char line[256];
    unsigned checksum = 0;
    for (const char *p = payload; *p; ++p) checksum ^= (unsigned char)*p;
    int length = snprintf(line, sizeof(line), "%s*%02X%s", payload, checksum, ending);
    VisionResult result;
    assert(vision_parse_line(line, (size_t)length, &result));
    return result;
}

static void invalid_payload(const char *payload)
{
    VisionResult result = parse_payload(payload, "");
    assert(result.status == VISION_INVALID_PACKET);
    assert(result.colour == VISION_COLOUR_NONE && result.marker_id == -1);
    assert(result.x_px == -1 && result.y_px == -1);
    assert(result.marker_width_px == 0 && result.distance_mm == 0);
}

int main(void)
{
    VisionResult result;
    assert(vision_parse_line("C,N*21", 6, &result));
    assert(result.status == VISION_NO_OBJECT && result.kind == VISION_COLOUR);
    assert(result.colour == VISION_COLOUR_NONE && result.marker_id == -1);
    assert(result.x_px == -1 && result.y_px == -1);
    result = parse_payload("C,N", "\r\n");
    assert(result.status == VISION_NO_OBJECT);

    const char *colours[] = {"C,R,160,120", "C,G,0,0", "C,B,319,239"};
    for (unsigned i = 0; i < 3; ++i) {
        result = parse_payload(colours[i], "\n");
        assert(result.status == VISION_OK && result.kind == VISION_COLOUR);
        assert(result.colour == (VisionColour)(VISION_RED + i));
        assert(result.marker_id == -1 && result.distance_mm == 0);
    }
    result = parse_payload("A,42,160,120,40.0,554.0", "");
    assert(result.status == VISION_OK && result.kind == VISION_ARUCO);
    assert(result.marker_id == 42 && result.x_px == 160 && result.y_px == 120);
    assert(result.marker_width_px == 40.0f && result.distance_mm == 554.0f);

    const char *bad[] = {
        "C,", "C,N,21,0", "C,R", "C,R,1", "C,R,,2", "C,X,1,2",
        "C,R,-1,2", "C,R,320,0", "C,R,0,240", "C,R,1,2,3",
        "C,R,999999999999999999999,2", "C,R,1.0,2", "C,R, 1,2",
        "A,42,160,120,40.0", "A,-1,1,2,3.0,4.0", "A,1,1,2,0.0,4.0",
        "A,1,1,2,3.0,0.0", "A,1,1,2,nan,4.0", "A,1,1,2,3.0,inf",
        "A,1,1,2,3.0,4.0,trailing", "A,1,1,2,3.,4.0"
    };
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) invalid_payload(bad[i]);
    const char *bad_wire[] = {
        "C,N", "C,N*20", "C,N*2", "C,N*021", "C,N*21junk", "C,N*ZZ",
        "C,N**21", "C,N*21\r\r", "C,N*21\nnoise"
    };
    for (unsigned i = 0; i < sizeof(bad_wire) / sizeof(bad_wire[0]); ++i) {
        assert(vision_parse_line(bad_wire[i], strlen(bad_wire[i]), &result));
        assert(result.status == VISION_INVALID_PACKET && result.x_px == -1);
    }
    assert(!vision_parse_line("boot log", 8, &result));
    assert(!vision_parse_line("log C,N*21", 10, &result));
    assert(!vision_parse_line(NULL, 0, &result));
    char overlong[200];
    memset(overlong, '1', sizeof(overlong)); overlong[0] = 'C'; overlong[1] = ',';
    assert(vision_parse_line(overlong, sizeof(overlong), &result));
    assert(result.status == VISION_INVALID_PACKET);

    /* Exercise length bounds with deterministic arbitrary bytes, including NUL. */
    unsigned state = 7;
    char fuzz[VISION_RX_LINE_SIZE];
    for (unsigned iteration = 0; iteration < 10000; ++iteration) {
        size_t length = iteration % sizeof(fuzz);
        for (size_t i = 0; i < length; ++i) {
            state = state * 1664525u + 1013904223u;
            fuzz[i] = (char)(state >> 24);
        }
        if (length >= 2) { fuzz[0] = 'C'; fuzz[1] = ','; }
        vision_parse_line(fuzz, length, &result);
    }
    puts("PASS: parser, C,N regression, units/fields, malformed packets, 10000 bounded inputs");
    return 0;
}
