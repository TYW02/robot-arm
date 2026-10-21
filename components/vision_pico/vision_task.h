#ifndef VISION_TASK_H
#define VISION_TASK_H

#include <tk/tkernel.h>
#include "vision.h"

typedef enum { VISION_MODE_SCAN, VISION_MODE_COLOUR, VISION_MODE_IDLE } VisionMode;

#ifdef __cplusplus
extern "C" {
#endif

/* Call once from usermain, before the single Coordinator consumer starts. */
ER VisionStart(void);
/* Nonblocking command queue. Only the Vision task touches the UART. */
ER VisionSetMode(VisionMode mode);
/* E_OK: result received (inspect status). E_TMOUT: status=VISION_TIMEOUT.
 * Other kernel errors: do not consume the result as a detection.
 * timeout uses the normal micro T-Kernel TMO convention (milliseconds).
 */
ER VisionReceive(VisionResult *result, TMO timeout);

#ifdef __cplusplus
}
#endif

#endif
