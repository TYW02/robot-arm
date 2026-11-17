/* Handler for commands received over Wi-Fi (UDP port 5006).
 *
 *   W  forward        S  backward
 *   A  turn left      D  turn right
 *   X  stop           R  hand control back to the autonomous mission
 *   PING              logged only, no movement
 *
 * W/A/S/D/X switch the car to manual mode (the autonomous mission's drive
 * commands are ignored until R).  Each W/A/S/D is a short pulse, so the car
 * stops by itself if the commands stop arriving. */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "wifi_log.h"

extern INT robot_logf(const UB *format, ...);

#define REMOTE_CMD_MAX   48U
#define REMOTE_SPEED     30   /* forward / backward, percent */
#define REMOTE_TURN      25   /* in-place turn, percent */
#define REMOTE_LOG_EVERY 10U  /* log every Nth repeat of the same key */

void robot_cmd_receive(const unsigned char *bytes, unsigned int length)
{
    static UB last_key;
    static unsigned int repeat;
    UB text[REMOTE_CMD_MAX + 1U];
    unsigned int n = 0;
    UB key;

    /* Copy printable characters only, so a stray byte cannot corrupt the log. */
    for(unsigned int i = 0; i < length && n < REMOTE_CMD_MAX; i++) {
        if(bytes[i] >= 0x20U && bytes[i] < 0x7FU) text[n++] = bytes[i];
    }
    text[n] = '\0';

    key = (n == 1U) ? text[0] : 0;
    if(key >= 'a' && key <= 'z') key = (UB)(key - 'a' + 'A');

    switch(key) {
    case 'W': robot_remote_drive( REMOTE_SPEED,  REMOTE_SPEED); break;
    case 'S': robot_remote_drive(-REMOTE_SPEED, -REMOTE_SPEED); break;
    case 'A': robot_remote_drive(-REMOTE_TURN,   REMOTE_TURN);  break;
    case 'D': robot_remote_drive( REMOTE_TURN,  -REMOTE_TURN);  break;
    case 'X': robot_remote_stop(); break;
    case 'R': robot_remote_auto(); break;
    default:  key = 0; break;
    }

    /* Key auto-repeat would flood the log: print a changed key at once and
     * a repeated one every few times. */
    if(key != 0 && key == last_key && ++repeat % REMOTE_LOG_EVERY != 0U) return;
    if(key != last_key) repeat = 0;
    last_key = key;

    if(key != 0) {
        robot_logf((UB*)"[cmd] %c\n", key);
    } else {
        robot_logf((UB*)"[cmd] received over Wi-Fi: \"%s\" (%u bytes)\n",
                   text, length);
    }
}
