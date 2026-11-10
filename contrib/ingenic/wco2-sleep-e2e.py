#!/usr/bin/env python3
"""
Run a Wyze Cam Outdoor v2 image through its sleep and wake cycle.

The stock app sleeps by itself when it has nothing to do: it asks the MCU
(wyze-wco2-mcu) to cut the SoC's power, the SoC stops, and an event the MCU
sees (the button, the PIR) raises the power again and the SoC boots from
cold. This script boots the image, waits for the app, then for its sleep,
and wakes it by each source in turn, checking after each wake that the SoC
cold-booted and that the app read the reason the MCU reported.

    wco2-sleep-e2e.py IMAGE [--wdt-scale N] [--log FILE]

A cycle takes about 10 minutes. The log is the console of the whole run.

Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MCU = "/machine/peripheral/mcu"

# What the app prints, and the reason it reads for each wake source
# (the MCU's event bits as the app's daemon maps them)
COLD = re.compile(r"Linux version")
REASON = re.compile(r"wakeupFlag : (0[xX][0-9A-Fa-f]+|0)")
SLEEP = re.compile(r"go_sleep_immediately")
REASONS = {"button": 0x1, "pir": 0x20}


class Console(threading.Thread):
    """The guest's console: everything it prints, with the time it came"""

    def __init__(self, path, log):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.lines = []
        self.partial = ""
        self.last = time.time()
        self.log = open(log, "w", errors="replace")
        self.sock = socket.socket(socket.AF_UNIX)
        for _ in range(600):
            try:
                self.sock.connect(path)
                break
            except OSError:
                time.sleep(0.1)
        else:
            raise RuntimeError("no console socket")

    def run(self):
        while True:
            try:
                data = self.sock.recv(4096)
            except OSError:
                return
            if not data:
                return
            text = data.decode("latin1").replace("\r", "")
            self.log.write(text)
            self.log.flush()
            with self.lock:
                self.last = time.time()
                self.partial += text
                *done, self.partial = self.partial.split("\n")
                self.lines.extend(done)

    def mark(self):
        with self.lock:
            return len(self.lines)

    def since(self, mark):
        with self.lock:
            return list(self.lines[mark:])

    def quiet_for(self):
        return time.time() - self.last


class Qmp:
    def __init__(self, path):
        self.sock = socket.socket(socket.AF_UNIX)
        for _ in range(600):
            try:
                self.sock.connect(path)
                break
            except OSError:
                time.sleep(0.1)
        else:
            raise RuntimeError("no QMP socket")
        self.f = self.sock.makefile("rw")
        self.f.readline()
        self.call("qmp_capabilities")

    def call(self, cmd, **args):
        self.f.write(json.dumps({"execute": cmd, "arguments": args}) + "\n")
        self.f.flush()
        while True:
            r = json.loads(self.f.readline())
            if "return" in r:
                return r["return"]
            if "error" in r:
                raise RuntimeError(r["error"]["desc"])

    def get(self, prop):
        return self.call("qom-get", path=MCU, property=prop)

    def set(self, prop, value):
        self.call("qom-set", path=MCU, property=prop, value=value)

    def pc(self):
        out = self.call("human-monitor-command", **{"command-line":
                                                   "info registers"})
        return re.search(r"PC\s+(0x[0-9a-f]+)", out).group(1)


class Retry(Exception):
    """A race in the guest's own start-up, not in the MCU: run again"""


DEAD = re.compile(r"Please run zrt_cam_daemon first")


def pair(src, dst):
    """Copy the image with a Wi-Fi name in the boot environment the SPL
    hands the kernel (senv;...eenv;): the app counts a camera as paired
    by it, and arms the PIR for its sleep only when paired"""
    img = bytearray(open(src, "rb").read())
    n = 0
    for m in list(re.finditer(rb"senv;[^\0]*?eenv;\0", img[:0x100000])):
        rec = m.group(0)[:-6] + b"[WIFI];SSID=wco2;eenv;\0"
        end = m.start() + len(rec)
        if img[m.end():end] != b"\xff" * (end - m.end()):
            raise SystemExit("no room in the boot environment")
        img[m.start():end] = rec
        n += 1
    if not n:
        raise SystemExit("no boot environment in the image")
    open(dst, "wb").write(img)


def say(msg):
    print("%s  %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def wait_for(what, test, timeout):
    end = time.time() + timeout
    while time.time() < end:
        r = test()
        if r:
            return r
        time.sleep(1)
    raise SystemExit("FAIL: %s not seen in %d s" % (what, timeout))


def find(con, mark, pattern):
    for line in con.since(mark):
        m = pattern.search(line)
        if m:
            return m
    return None


def main():
    for attempt in range(3):
        if attempt:
            say("run %d" % (attempt + 1))
        if not once():
            return 0
    raise SystemExit("FAIL: the guest's start-up raced 3 times")


def once():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("image")
    ap.add_argument("--wdt-scale", type=int, default=1,
                    help="stretch the MCU's watchdog on a slow host")
    ap.add_argument("--log", default="wco2-sleep-e2e.log")
    ap.add_argument("--boot-timeout", type=int, default=900)
    ap.add_argument("--extra", default="",
                    help="more arguments for the SoC's QEMU")
    ap.add_argument("--qemu-bsp", default=os.path.join(HERE, "qemu-bsp.sh"))
    args = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="wco2-")
    image = tmp + "/image"
    pair(args.image, image)
    con_path, qmp_path = tmp + "/con", tmp + "/qmp"
    # the board of README.md's "Wyze Cam Outdoor v2 MCU" section
    extra = (
        "-qmp unix:%s,server=on,wait=off "
        "-device image-sensor,model=gc2063,bus=i2c0 "
        "-device bcm43438,bus=msc1,netdev=w0,reg-on-pin=-1 "
        "-netdev user,id=w0 "
        "-device ingenic-gpio-i2c,id=i2c-gpio1,sda=57,scl=58 "
        "-device cw2015,bus=i2c-gpio1,voltage-mv=3900 "
        "-device ingenic-gpio-i2c,id=i2c-gpio2,sda=54,scl=51 "
        "-device wyze-wco2-mcu,bus=i2c-gpio2,id=mcu,wdt-scale=%d"
    ) % (qmp_path, args.wdt_scale) + " " + args.extra
    proc = subprocess.Popen(
        [args.qemu_bsp, "-i", image, "-V", "t31zx", "-m", "128M", "-R",
         "--no-ssh", "-c", con_path, "-E", extra],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    try:
        run(args, proc, con_path, qmp_path)
        return 0
    except Retry as e:
        say("retry: %s" % e)
        return 1
    finally:
        # the SoC's QEMU and the RISC-V core's share the session
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait()
        shutil.rmtree(tmp, ignore_errors=True)


def run(args, proc, con_path, qmp_path):
    con = Console(con_path, args.log)
    con.start()
    qmp = Qmp(qmp_path)
    bt = args.boot_timeout

    def boot_up(mark, what, first=False):
        if not first:
            wait_for("a cold boot", lambda: find(con, mark, COLD), bt)
        # the app reads the reason from the MCU once its daemon is up; on
        # some boots the daemon misses its D-Bus registration and never is
        def up():
            if sum(1 for l in con.since(mark) if DEAD.search(l)) > 30:
                raise Retry("the app's daemon did not start")
            return find(con, mark, REASON)

        m = wait_for("the app up", up, bt)
        say("%s: cold boot, app up, wake reason read: %s" %
            (what, m.group(1) if m else "none"))
        return m

    def asleep(mark, what):
        wait_for("the app's sleep", lambda: find(con, mark, SLEEP), bt)
        wait_for("the power master off",
                 lambda: not qmp.get("out-master"), bt)
        pc = qmp.pc()
        time.sleep(5)
        stopped = qmp.pc() == pc and con.quiet_for() >= 4
        pir = qmp.get("out-pir")
        say("%s: power master off, SoC %s, PIR %s" %
            (what, "stopped" if stopped else "STILL RUNNING",
             "armed" if pir else "off"))
        if not stopped:
            raise SystemExit("FAIL: the SoC did not stop")
        return pir

    def press(prop, value):
        qmp.set(prop, value)

    def wake(source):
        mark = con.mark()
        if source == "button":
            press("button", True)
            time.sleep(0.5)
            press("button", False)
        else:
            press("pir-adc", 3500)
            press("pir", True)
            time.sleep(0.5)
            press("pir", False)
        wait_for("the power master on", lambda: qmp.get("out-master"), 120)
        m = boot_up(mark, "wake by " + source)
        want = REASONS[source]
        got = int(m.group(1), 16) if m else None
        if got is None or not got & want:
            raise SystemExit("FAIL: the app read %s, wanted bit 0x%x" %
                             (m.group(1) if m else "no reason", want))
        return mark

    boot_up(0, "boot", True)
    if not asleep(0, "sleep 1"):
        raise Retry("the app did not arm the PIR")
    mark = wake("pir")
    for line in con.since(mark):
        if re.search(r"camera task|Real-time alarm|check_pir", line):
            say("    app: " + line.strip()[:100])
    asleep(mark, "sleep 2")
    wake("button")
    say("PASS")


if __name__ == "__main__":
    sys.exit(main())
