#!/usr/bin/env python3
"""Send one operator command to the Pico W over Wi-Fi and print its answer.

    python3 tools/wifi_cmd_send.py <pico-ip> <command...>

Examples:
    python3 tools/wifi_cmd_send.py 172.20.10.11 status
    python3 tools/wifi_cmd_send.py 172.20.10.11 start
    python3 tools/wifi_cmd_send.py 172.20.10.11 f 40 300

The commands are the same as on the USB console (type "help" for the list).
"""
import socket
import sys

PORT = 5006


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    ip = sys.argv[1]
    command = " ".join(sys.argv[2:])
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    try:
        sock.sendto(command.encode("ascii", "replace"), (ip, PORT))
        reply, _ = sock.recvfrom(2048)
    except socket.timeout:
        print(f"No reply from {ip}:{PORT} within 2 s (is the Pico joined and the IP right?)")
        return 1
    except socket.gaierror:
        print(f"'{ip}' is not an IP address. Use the number from the [wifi-log] own ... line.")
        return 2
    print(reply.decode("utf-8", "replace").rstrip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
