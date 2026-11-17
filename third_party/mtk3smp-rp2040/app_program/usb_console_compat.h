/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	usb_console_compat.h
 *	Console-state accessors, with UART-build fallbacks.
 *
 *	The USB-CDC console exports real state; the UART build has none, so it
 *	gets constant stubs that read as "up, nothing pending".  Shared by
 *	every harness translation unit, since the reporting and blink paths
 *	both consult console state.
 */

#ifndef USB_CONSOLE_COMPAT_H
#define USB_CONSOLE_COMPAT_H

#include <tk/tkernel.h>

#if TM_CONSOLE_USB_CDC
IMPORT UW		tm_usb_state(void);	/* >= 2 once enumerated */
IMPORT INT		tm_usb_diag_level(void);/* bring-up level, 0..8 */
IMPORT UW		tm_usb_pending(void);	/* bytes still queued   */
IMPORT volatile UW	tm_usb_dropped;		/* bytes lost to overrun*/
Inline UW tm_usb_dropped_bytes(void)	{ return tm_usb_dropped; }
IMPORT INT		tm_console_getc(void);	/* -1 if no byte waiting */
IMPORT volatile UW	tm_usb_rx_dropped;	/* input lost to overrun */
Inline UW tm_usb_rx_dropped_bytes(void)	{ return tm_usb_rx_dropped; }
#else
/* UART build: tm_printf writes straight to the UART from the calling context.
   There is no ring, so it is always "up", never pending and never drops. */
Inline UW  tm_usb_state(void)		{ return 3; }
Inline INT tm_usb_diag_level(void)	{ return 8; }
Inline UW  tm_usb_pending(void)		{ return 0; }
Inline UW  tm_usb_dropped_bytes(void)	{ return 0; }
Inline UW  tm_usb_rx_dropped_bytes(void)	{ return 0; }
/* Non-blocking UART0 read (FR.RXFE = bit 4), unlike tm_getchar(). */
Inline INT tm_console_getc(void)
{
	if((*(volatile UW *)0x40034018) & (1U << 4)) return -1;
	return (INT)((*(volatile UW *)0x40034000) & 0xffU);
}
#endif

#endif /* USB_CONSOLE_COMPAT_H */
