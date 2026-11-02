#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""What a camera's pins and PWM outputs did, from QEMU's pin traces.

Run QEMU with the GPIO, timer unit and PWM block traces into files:

    -chardev file,id=gpio,path=gpio.jsonl -global ingenic-gpio.trace=gpio
    -chardev file,id=tcu,path=tcu.jsonl -global ingenic-tcu.trace=tcu
    -chardev file,id=pwm,path=pwm.jsonl -global ingenic-pwm.trace=pwm

Then, with any of them:

    pin-trace.py log gpio.jsonl pwm.jsonl         each change, in time order
    pin-trace.py vcd gpio.jsonl pwm.jsonl -o pins.vcd
    pin-trace.py serve gpio.jsonl pwm.jsonl       a live page, following them

A PWM output's line gives its level and, while it toggles, its period, high
time and next edge, so vcd draws every edge between one line and the next,
beside the channel's duty as a real signal.
"""

import argparse
import json
import os
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "pin-trace.html")


def read_lines(path, follow=False, stop=None):
    """A trace file's JSON lines as dicts, waiting for more while following"""
    with open(path, errors="replace") as f:
        buf = ""
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                if not follow or (stop and stop.is_set()):
                    break
                time.sleep(0.2)
                continue
            buf += chunk
            *lines, buf = buf.split("\n")
            for ln in lines:
                try:
                    yield json.loads(ln)
                except ValueError:
                    continue


def signal(e):
    """The signal a line is about, or None for a reset"""
    if "pin" in e:
        return e["pin"]
    for dev in ("tcu", "pwm"):
        if dev in e:
            return f"{dev}{e[dev]}"
    return None


def order(name):
    """Pins first, then the outputs, each by number: PB2 before PB10"""
    m = re.match(r"(\D+)(\d+)$", name)
    return (not name.startswith("P"), m.group(1), int(m.group(2))) if m \
        else (True, name, 0)


def events(paths):
    """Every file's lines, merged in time order"""
    out = []
    for i, p in enumerate(paths):
        for n, e in enumerate(read_lines(p)):
            out.append((e.get("t", 0), i, n, e))
    out.sort(key=lambda x: x[:3])
    return [e for *_, e in out]


def describe(e):
    if "pin" in e:
        fn = e["fn"]
        if fn == "func":
            what = f"function {e['func']}"
        elif fn == "irq":
            what = f"irq {e['trig']}{' masked' if e['mask'] else ''} " \
                   f"level {e['lvl']}"
        else:
            what = f"{fn} {e['lvl']}"
    elif "period" in e:
        hz = 1e9 / e["period"] if e["period"] else 0
        what = f"{hz:.6g} Hz, {100.0 * e['high'] / e['period']:.4g}% high, " \
               f"now {e['lvl']}, an edge in {e['edge']} ns"
    else:
        what = f"steady {e['lvl']}"
    by = e.get("pc") or e.get("src", "")
    return what, by


def cmd_log(a):
    for e in events(a.traces):
        t = e.get("t", 0) / 1e9
        name = signal(e)
        if name is None:
            print(f"{t:14.9f}  reset")
            continue
        what, by = describe(e)
        print(f"{t:14.9f}  {name:8} {what:60} {by}")
    return 0


def vcd_id(n):
    code = ""
    while True:
        n, r = divmod(n, 94)
        code = chr(33 + r) + code
        if not n:
            return code
        n -= 1


def pwm_edges(e, until, budget):
    """The edges of the waveform a PWM line starts, up to until: (ns, level)"""
    out = []
    lvl = e["lvl"]
    if not e.get("period"):
        return out, budget
    t = e["t"] + e["edge"]
    while t < until and budget > 0:
        lvl ^= 1
        out.append((t, lvl))
        t += e["high"] if lvl else e["period"] - e["high"]
        budget -= 1
    return out, budget


def cmd_vcd(a):
    evs = events(a.traces)
    if not evs:
        print("no events", file=sys.stderr)
        return 1
    end = max(e.get("t", 0) for e in evs) + a.tail
    changes = []
    names = []
    last = {}
    for e in evs:
        name = signal(e)
        if name is None:
            continue
        if name not in names:
            names.append(name)
        last.setdefault(name, []).append(e)
    budget = a.max_edges
    clipped = []
    names.sort(key=order)
    for name in names:
        lines = last[name]
        for i, e in enumerate(lines):
            t = e.get("t", 0)
            if "pin" in e:
                v = "z" if e["fn"] == "func" else str(e["lvl"])
                changes.append((t, name, v))
                continue
            changes.append((t, name, str(e["lvl"])))
            duty = e["high"] / e["period"] if e.get("period") else e["lvl"]
            changes.append((t, name + "_duty", f"r{duty:.6g}"))
            until = lines[i + 1]["t"] if i + 1 < len(lines) else end
            edges, budget = pwm_edges(e, until, budget)
            if e.get("period") and budget <= 0 and name not in clipped:
                clipped.append(name)
            changes.extend((t2, name, str(v)) for t2, v in edges)
    ids = {}
    with open(a.output, "w") as f:
        f.write("$timescale 1ns $end\n$scope module pins $end\n")
        for name in names:
            ids[name] = vcd_id(len(ids))
            f.write(f"$var wire 1 {ids[name]} {name} $end\n")
            if not name.startswith("P"):
                ids[name + "_duty"] = vcd_id(len(ids))
                f.write(f"$var real 1 {ids[name + '_duty']} {name}_duty $end\n")
        f.write("$upscope $end\n$enddefinitions $end\n")
        changes.sort(key=lambda c: c[0])
        now = None
        for t, name, v in changes:
            if t != now:
                f.write(f"#{t}\n")
                now = t
            if v.startswith("r"):
                f.write(f"{v} {ids[name]}\n")
            else:
                f.write(f"{v}{ids[name]}\n")
        f.write(f"#{end}\n")
    if clipped:
        print(f"{', '.join(clipped)}: edges past --max-edges {a.max_edges} "
              f"left out", file=sys.stderr)
    return 0


def cmd_serve(a):
    lock = threading.Lock()
    stop = threading.Event()
    live = {"seq": 0, "t": 0, "signals": {}, "recent": []}

    def follow(path):
        for e in read_lines(path, follow=True, stop=stop):
            with lock:
                live["seq"] += 1
                live["t"] = max(live["t"], e.get("t", 0))
                name = signal(e)
                if name is None:
                    live["recent"].append([e.get("t", 0), "reset", "", ""])
                    continue
                what, by = describe(e)
                live["signals"][name] = dict(e, name=name, what=what, by=by)
                live["recent"].append([e.get("t", 0), name, what, by])
                del live["recent"][:-300]

    def state():
        with lock:
            return {"seq": live["seq"], "t": live["t"],
                    "signals": sorted(live["signals"].values(),
                                      key=lambda s: order(s["name"])),
                    "recent": sorted(live["recent"],
                                     key=lambda r: r[0])[-200:]}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def reply(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/":
                with open(PAGE, "rb") as f:
                    self.reply(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/meta":
                self.reply(200, json.dumps({"traces": a.traces}))
            elif self.path == "/events":
                self.events()
            else:
                self.reply(404, '{"error":"not found"}')

        def events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq, quiet = -1, time.time()
            try:
                while True:
                    if live["seq"] != seq:
                        st = state()
                        seq = st["seq"]
                        self.wfile.write(f"event: state\ndata: "
                                         f"{json.dumps(st)}\n\n".encode())
                        self.wfile.flush()
                        quiet = time.time()
                    elif time.time() - quiet > 10:
                        self.wfile.write(b": still here\n\n")
                        self.wfile.flush()
                        quiet = time.time()
                    time.sleep(a.interval)
            except (BrokenPipeError, ConnectionResetError):
                pass

    for p in a.traces:
        threading.Thread(target=follow, args=(p,), daemon=True).start()
    httpd = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    httpd.daemon_threads = True
    print(f"http://127.0.0.1:{httpd.server_address[1]}/  "
          f"({', '.join(a.traces)}; Ctrl-C stops)", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    stop.set()
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                formatter_class=argparse.RawTextHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("log", help="each change, in time order")
    sp.add_argument("traces", nargs="+")
    sp.set_defaults(fn=cmd_log)

    sp = sub.add_parser("vcd", help="the pins and PWM outputs as waveforms")
    sp.add_argument("traces", nargs="+")
    sp.add_argument("-o", "--output", required=True)
    sp.add_argument("--max-edges", type=int, default=1000000,
                    help="PWM edges drawn in all (1000000)")
    sp.add_argument("--tail", type=int, default=0,
                    help="ns to draw past the last line (0)")
    sp.set_defaults(fn=cmd_vcd)

    sp = sub.add_parser("serve", help="a live page, following the traces")
    sp.add_argument("traces", nargs="+")
    sp.add_argument("--port", type=int, default=0)
    sp.add_argument("--interval", type=float, default=0.5,
                    help="seconds between updates (0.5)")
    sp.set_defaults(fn=cmd_serve)

    a = p.parse_args()
    try:
        return a.fn(a)
    except BrokenPipeError:
        return 0


if __name__ == "__main__":
    sys.exit(main())
