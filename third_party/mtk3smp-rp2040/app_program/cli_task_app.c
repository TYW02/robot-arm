/* Build the project-owned CLI task (UI Task) as its own translation unit.
   cli_utkernel.h first: it selects the standard size_t before the kernel
   headers; usb_console_compat.h lives here in app_program. */
#include "../../../components/cli/cli_utkernel.h"
#include "usb_console_compat.h"
#include "../../../components/cli/cli_utkernel.c"
