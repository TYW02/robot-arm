/* Stage 1 integration entry point: Vision -> Coordinator, no Motion linked. */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../components/vision_pico/vision_task.h"
#include "../components/state_machine/coordinator.h"

EXPORT INT usermain(void)
{
    tm_printf((UB *)"Vision interface test: MOTION DISABLED\n");
    ER error = VisionStart();
    if (error < E_OK) {
        tm_printf((UB *)"[init] VisionStart failed: %d\n", error);
        return 1;
    }
    error = CoordinatorStart();
    if (error < E_OK) {
        tm_printf((UB *)"[init] Coordinator start failed: %d\n", error);
        return 1;
    }
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
