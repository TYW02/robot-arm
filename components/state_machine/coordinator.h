#ifndef SYSTEM_COORDINATOR_H
#define SYSTEM_COORDINATOR_H

#include <tk/tkernel.h>

/* Call after VisionStart. This stage only receives/prints VisionResult. */
ER CoordinatorStart(void);

#endif
