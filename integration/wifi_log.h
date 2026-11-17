#ifndef ROBOT_WIFI_LOG_H
#define ROBOT_WIFI_LOG_H

int robot_log_dequeue(unsigned char *bytes, unsigned int capacity,
                      unsigned int *length);

/* Called from the Wi-Fi radio task for every UDP command datagram received
 * on port 5006.  Keep it short: it runs inside the lwIP/CYW43 polling loop. */
void robot_cmd_receive(const unsigned char *bytes, unsigned int length);

/* Remote drive, implemented in main.c.  Never block; safe to call from the
 * radio task.  left/right are wheel speeds in percent (-100..100). */
void robot_remote_drive(int left, int right);
void robot_remote_stop(void);
void robot_remote_auto(void);

#endif
