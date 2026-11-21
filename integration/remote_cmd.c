/* Operator commands received over Wi-Fi (UDP port 5006).
 *
 * Each datagram is one command line, the same text as on the USB console:
 *
 *   start | stop (x) | reset | status | help (?) | stats
 *   f|b|l|r [speed] [ms] | drive <L> <R> [ms] | open | close | claw <0-180>|off
 *
 * It goes through the same parser (components/cli) and the same handlers
 * (integration/cli_bridge.c) as the USB CLI, so the start/stop rules, the
 * IDLE-only debug commands and the IMU safety interlock apply unchanged.
 * Runs in the Wi-Fi radio task and never blocks. */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "wifi_log.h"

extern INT robot_logf(const UB *format, ...);
extern int cli_utkernel_exec_line(const char *line, char *out, unsigned int out_len);

#define REMOTE_LINE_MAX 64U   /* longer than the parser's limit, so it can say "too long" */

int robot_cmd_receive(const unsigned char *bytes, unsigned int length,
                      char *reply, unsigned int reply_max)
{
    char text[REMOTE_LINE_MAX + 1U];
    unsigned int n = 0;
    int reply_len;

    /* Printable characters only: drops the newline a sender may add and any
     * stray byte that could corrupt the log. */
    for(unsigned int i = 0; i < length && n < REMOTE_LINE_MAX; i++) {
        if(bytes[i] >= 0x20U && bytes[i] < 0x7FU) text[n++] = (char)bytes[i];
    }
    text[n] = '\0';

    reply[0] = '\0';
    if(n == 0U) return 0;                 /* blank datagram: nothing to do */
    reply_len = cli_utkernel_exec_line(text, reply, reply_max);

    /* Log the command and the first line of the answer. */
    {
        char first[81];
        unsigned int k = 0;
        while(reply[k] != '\0' && reply[k] != '\n' && k < 80U) { first[k] = reply[k]; k++; }
        first[k] = '\0';
        robot_logf((UB*)"[cmd] \"%s\" -> %s\n", text, first);
    }
    return reply_len;
}
