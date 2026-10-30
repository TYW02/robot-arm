#ifndef ROBOT_WIFI_LOG_H
#define ROBOT_WIFI_LOG_H

int robot_log_dequeue(unsigned char *bytes, unsigned int capacity,
                      unsigned int *length);

#endif
