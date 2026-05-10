#!/usr/bin/env python3
"""
Drive a QEMU UART console: connect once, send all commands from stdin,
append output to a log file, exit.

Pairs with qemu-bsp.sh -c PATH (UNIX socket) or -T PORT (TCP).

Usage:
  echo "uname -a" | qemu_drive.py /tmp/qemu.sock     # UNIX socket
  echo "uname -a" | qemu_drive.py 127.0.0.1:3331     # TCP

Environment:
  QEMU_DRIVE_LOG     log file to append output to (default /tmp/qemu-console.out)
  QEMU_DRIVE_IDLE    seconds of quiet before next command (default 1.5)

Example:
  qemu-bsp.sh -i /tmp/thingino-X.bin -c /tmp/q.sock &
  cat <<EOF | qemu_drive.py /tmp/q.sock
  /usr/sbin/usb-role -m host
  sleep 4
  ls /sys/bus/usb/devices/
  EOF
"""
import os
import select
import socket
import sys
import time


def open_endpoint(spec: str) -> socket.socket:
    """Open a UNIX-socket path or 'host:port' TCP endpoint."""
    if ":" in spec and "/" not in spec:
        host, port = spec.rsplit(":", 1)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, int(port)))
    else:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(spec)
    s.setblocking(False)
    return s


def drain(s: socket.socket, log, secs: float) -> None:
    """Read from socket for `secs` of quiet (extending on each data arrival)."""
    end = time.time() + secs
    while time.time() < end:
        r, _, _ = select.select([s], [], [], 0.2)
        if r:
            try:
                data = s.recv(65536)
                if data:
                    log.write(data)
                    log.flush()
                    end = time.time() + secs
            except BlockingIOError:
                pass


def main() -> int:
    if len(sys.argv) != 2:
        sys.stderr.write(__doc__)
        return 1

    log_path = os.environ.get("QEMU_DRIVE_LOG", "/tmp/qemu-console.out")
    idle = float(os.environ.get("QEMU_DRIVE_IDLE", "1.5"))

    cmds = sys.stdin.read().splitlines()
    s = open_endpoint(sys.argv[1])

    with open(log_path, "ab") as log:
        log.write(b"\n=== qemu_drive start ===\n")
        log.flush()

        # Wake up the shell.
        s.send(b"\n")
        time.sleep(0.3)
        drain(s, log, 0.5)

        for cmd in cmds:
            cmd = cmd.strip()
            if not cmd:
                continue
            s.send(cmd.encode() + b"\n")
            wait = idle
            if cmd.startswith("sleep "):
                try:
                    wait = float(cmd.split()[1]) + 1.0
                except (ValueError, IndexError):
                    pass
            drain(s, log, wait)

        drain(s, log, 1.0)

    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
