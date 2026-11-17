#!/usr/bin/env python3
"""Send one command to the Pico W over UDP (port 5006) and wait for its reply.

Usage:  python3 tools/wifi_cmd_send.py <pico-ip> <command>
Example: python3 tools/wifi_cmd_send.py 172.20.10.5 PING

The Pico IP is printed on the USB console in the [wifi-log] line ("own x.x.x.x").
"""

import socket
import sys

PORT = 5006
TIMEOUT_S = 2.0


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    address, command = sys.argv[1], " ".join(sys.argv[2:])
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.settimeout(TIMEOUT_S)
    try:
        sender.sendto(command.encode("utf-8"), (address, PORT))
        print(f"Sent {command!r} to {address}:{PORT}")
        try:
            reply, _ = sender.recvfrom(256)
            print(f"Pico replied: {reply.decode('utf-8', errors='replace').strip()!r}")
            return 0
        except socket.timeout:
            print(f"No reply within {TIMEOUT_S:.0f} s (the Pico may still have received it; "
                  "check the console for a [cmd] line).")
            return 1
    finally:
        sender.close()


if __name__ == "__main__":
    raise SystemExit(main())
