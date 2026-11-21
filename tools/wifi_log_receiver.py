#!/usr/bin/env python3
"""Print UDP log datagrams from the Pico W (port 5005)."""

import socket
import sys


PORT = 5005


def main() -> int:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receiver.bind(("0.0.0.0", PORT))
    print(f"Listening for Pico W logs on UDP port {PORT}...", flush=True)
    pending: dict[str, str] = {}

    try:
        while True:
            payload, (address, _port) = receiver.recvfrom(2048)
            text = pending.get(address, "") + payload.decode("utf-8", errors="replace")
            while "\n" in text:
                line, text = text.split("\n", 1)
                print(f"[Pico {address}] {line}", flush=True)
            pending[address] = text
    except KeyboardInterrupt:
        for address, text in pending.items():
            if text:
                print(f"[Pico {address}] {text}")
        print("\nStopped.")
    finally:
        receiver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
