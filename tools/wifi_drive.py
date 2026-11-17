#!/usr/bin/env python3
"""Drive the Pico W car from the keyboard over Wi-Fi (UDP port 5006).

Usage:  python3 tools/wifi_drive.py <pico-ip>

  W forward   S backward   A turn left   D turn right
  X or SPACE  stop         R give control back to the autonomous mission
  Q           stop and quit

Press a key once and the car keeps going: the last direction is re-sent 10 times a
second until you press another key or X. Each command only moves the car for
~150 ms, so if this script stops or Wi-Fi drops, the car stops by itself.
"""

import select
import socket
import sys
import termios
import time
import tty

PORT = 5006
SEND_INTERVAL_S = 0.1
KEYS = {"w": "W", "a": "A", "s": "S", "d": "D", "x": "X", " ": "X", "r": "R"}


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    target = (sys.argv[1], PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    current = ""          # direction being repeated (W/A/S/D), or "" for none
    replies = 0
    last_sent = 0.0
    print("Driving. W/A/S/D move, X stop, R auto, Q quit.\r")
    try:
        tty.setcbreak(fd)
        while True:
            ready, _, _ = select.select([sys.stdin, sock], [], [], SEND_INTERVAL_S)
            if sock in ready:
                try:
                    while sock.recv(64):
                        replies += 1
                except BlockingIOError:
                    pass
            if sys.stdin in ready:
                key = sys.stdin.read(1).lower()
                if key == "q":
                    break
                command = KEYS.get(key)
                if command is not None:
                    current = command if command in "WASD" else ""
                    sock.sendto(command.encode(), target)
                    last_sent = time.monotonic()
                    print(f"sent {command}   (replies so far: {replies})\r")
                    continue
            # Re-send the current direction on a timer, not on every wake-up:
            # the Pico's replies also wake this loop.
            if current and time.monotonic() - last_sent >= SEND_INTERVAL_S:
                sock.sendto(current.encode(), target)
                last_sent = time.monotonic()
    finally:
        sock.sendto(b"X", target)
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        sock.close()
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
