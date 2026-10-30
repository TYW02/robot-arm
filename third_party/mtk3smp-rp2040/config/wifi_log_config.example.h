#ifndef WIFI_LOG_CONFIG_H
#define WIFI_LOG_CONFIG_H

/* Set these to the laptop's Wi-Fi IPv4 address running wifi_log_receiver.py. */
#define UTK_WIFI_LOG_TARGET_A 192
#define UTK_WIFI_LOG_TARGET_B 168
#define UTK_WIFI_LOG_TARGET_C 1
#define UTK_WIFI_LOG_TARGET_D 100

/* 1 = send to the whole subnet (broadcast), so the laptop IP above is ignored
 * and nothing needs rebuilding when DHCP changes it.  0 = send only to the IP
 * above.  Some hotspots and campus networks block broadcast; use 0 there. */
#define UTK_WIFI_LOG_BROADCAST 1

#endif /* WIFI_LOG_CONFIG_H */
