#ifndef USB_CONSOLE_COMPAT_H
#define USB_CONSOLE_COMPAT_H

#include <tk/tkernel.h>

#if TM_CONSOLE_USB_CDC

IMPORT UW tm_usb_state(void);
IMPORT INT tm_usb_diag_level(void);
IMPORT UW tm_usb_pending(void);
IMPORT volatile UW tm_usb_dropped;

Inline UW tm_usb_dropped_bytes(void)
{
    return tm_usb_dropped;
}

#else

Inline UW tm_usb_state(void)
{
    return 3;
}

Inline INT tm_usb_diag_level(void)
{
    return 8;
}

Inline UW tm_usb_pending(void)
{
    return 0;
}

Inline UW tm_usb_dropped_bytes(void)
{
    return 0;
}

#endif

#endif
