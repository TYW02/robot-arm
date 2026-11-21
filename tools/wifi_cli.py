#!/usr/bin/env python3
"""Interactive operator console for the Pico W over Wi-Fi.

    python3 tools/wifi_cli.py <pico-ip>

Type the same commands as on the USB console (start, stop, reset, status,
f/b/l/r, drive, open, close, claw, help) and press Enter. Type "quit" to leave.
Leaving does not stop the car: send "stop" first if it is moving.
"""
import socket
import sys

PORT = 5006


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    ip = sys.argv[1]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    print(f"Connected to {ip}:{PORT}. Type 'help' for commands, 'quit' to leave.")
    while True:
        try:
            line = input("pico> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not line:
            continue
        if line.lower() in ("quit", "exit"):
            return 0
        try:
            sock.sendto(line.encode("ascii", "replace"), (ip, PORT))
            reply, _ = sock.recvfrom(2048)
            print(reply.decode("utf-8", "replace").rstrip())
        except socket.timeout:
            print("(no reply within 2 s)")
        except socket.gaierror:
            print(f"'{ip}' is not an IP address.")
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
