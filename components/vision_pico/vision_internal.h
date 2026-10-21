#ifndef VISION_INTERNAL_H
#define VISION_INTERNAL_H

#include <stdbool.h>
#include "vision.h"

#define VISION_RX_LINE_SIZE 128

void vision_clear_result(VisionResult *result, VisionStatus status);
/* false: unrelated boot/log line, ignored. true: result includes status.
 * Length-bounded parsing keeps the checksum outside all payload fields.
 */
bool vision_parse_line(const char *line, unsigned length, VisionResult *result);

#endif
