#ifndef ROBOT_WIFI_LOG_H
#define ROBOT_WIFI_LOG_H

int robot_log_dequeue(unsigned char *bytes, unsigned int capacity,
                      unsigned int *length);

/* Called from the Wi-Fi radio task for every UDP datagram received on the
 * command port (5006).  The datagram is one operator command line, exactly as
 * it would be typed on the USB console (start, stop, reset, status, help,
 * f/b/l/r, drive, open, close, claw, stats).  The text for the sender is
 * written to reply (NUL-terminated); returns its length, 0 = no reply. */
int robot_cmd_receive(const unsigned char *bytes, unsigned int length,
                      char *reply, unsigned int reply_max);

#endif
