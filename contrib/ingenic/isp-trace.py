#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""What a camera's ISP and image sensor did, from a QEMU trace.

Run QEMU with the ISP's and the sensor's trace events into a file:

    -msg timestamp=on -trace 'ingenic_isp_*' -trace 'image_sensor_*' -D isp.log

(ingenic_t20_isp_* and ingenic_t30_ncu_* for a T10, T20 or T30's Apical
ISP; the T30's multi-scaler is ingenic_isp_*). Leave out the *_read events
for a log a tenth the size. Then:

    isp-trace.py log isp.log               what changed, frame by frame
    isp-trace.py snapshot isp.log --at frame:30
    isp-trace.py diff stock.log open.log --at stream
    isp-trace.py vcd isp.log -o isp.vcd    the registers as waveforms
    isp-trace.py serve isp.log             a live page, following the file

A register's state here is what was last written to it: by the driver,
through the shadow port of (value, offset) pairs or by a register list the
core loads. Snapshots and diffs leave out the registers that are not
state (status, clears, FIFO pushes, ports) unless --all.
"""

import argparse
import json
import os
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LINE = re.compile(r"(?:(\d{4}-\d\d-\d\d)T(\d\d):(\d\d):(\d\d\.\d+)Z |"
                  r"\d+@(\d+\.\d+):)?(\w+) ?(.*)")
ACCESS = re.compile(r"0x([0-9a-f]+) (->|<-|\|=) 0x([0-9a-f]+)"
                    r"(?: at 0x([0-9a-f]+))? ?(\S*)$")
SENSOR = re.compile(r"input (\d+) 0x([0-9a-f]+) (<-|->) 0x([0-9a-f]+)$")
FRAME = re.compile(r"VIC (\d+) frame (\d+): pipeline (\d+), input (\d+)")
APICAL_FRAME = re.compile(r"frame (\d+) (starts|out)")
VIC = re.compile(r"VIC (\d+) (start|stop)")
APICAL_VIC = re.compile(r"VIC (start|stop)")
EXPOSURE = re.compile(r"input (\d+) exposure (\d+)/16 lines, gain (\d+)/1024")
STREAM = re.compile(r"input (\d+) streaming (\d)")

ACCESS_EVENTS = {
    "ingenic_isp_read": ("isp", "read"),
    "ingenic_isp_write": ("isp", "write"),
    "ingenic_isp_shadow": ("isp", "shadow"),
    "ingenic_isp_rcfg_write": ("isp", "list"),
    "ingenic_isp_raise": ("isp", "raise"),
    "ingenic_t20_isp_read": ("apical", "read"),
    "ingenic_t20_isp_write": ("apical", "write"),
    "ingenic_t30_ncu_write": ("ncu", "write"),
}

# What the log view shows of the events that are not register accesses
NOTES = re.compile(r"ingenic_(isp|t20_isp)_(vic|vic_wait|chan_drop|chan_clear|"
                   r"clock|reset|csi|mclk|switch|rcfg)$|image_sensor_stream$")
CHANNEL = re.compile(r"ingenic_isp_(chan_push|chan_done|chan_pop|ivdc)$|"
                     r"ingenic_t20_isp_writer$")
IRQ = re.compile(r"ingenic_(isp|t20_isp)_(irq|vic_irq|event)$")

# Registers that hold no state: status, clears, FIFOs, ports, read-only
VOLATILE = re.compile(
    r"(^|\.)(int_sta1?|int_clr1?|int_msk1?|done_(sta|clr|y|uv|frame)|"
    r"tick_(sta|clr)|y_push|uv_push|clr_(y|uv|done_y|done_uv)|shadow|"
    r"shadow_open|rcfg_trig|rcfg_trig_hi|ch_busy|push_sta|busy|idle|ready|"
    r"base_curr|base_last|count)$|^(top\.(clr|clr_all|sta)|"
    r"core\.(int_sta|port_sta))$")


def events(path, follow=False, stop=None):
    """The trace's events in order, as dicts: "seq", "t" (seconds, or None
    without -msg timestamp=on), "ev", "args", and per kind "dev" plus
    "op", "off", "val", "name" (register accesses), "vic" and "frame"
    (frames), "lines" and "gain" (exposure)"""
    day, last_t, base = None, None, 0.0
    seq = 0
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
                m = LINE.match(ln)
                if not m:
                    continue
                d, hh, mm, ss, pidt, ev, args = m.groups()
                if ev not in ACCESS_EVENTS and not ev.startswith(
                        ("ingenic_isp_", "ingenic_t20_isp_", "image_sensor_",
                         "ingenic_t30_ncu_")):
                    continue
                t = None
                if d:
                    if day is not None and d != day:
                        base += 86400.0
                    day = d
                    t = base + int(hh) * 3600 + int(mm) * 60 + float(ss)
                elif pidt:
                    t = float(pidt)
                if t is not None:
                    last_t = t
                seq += 1
                e = {"seq": seq, "t": t if t is not None else last_t,
                     "ev": ev, "args": args}
                parse(e)
                yield e


PREFIX = re.compile(r"^(ingenic_(isp|t20_isp|t30_ncu)|image_sensor)_")


def parse(e):
    ev, args = e["ev"], e["args"]
    e["dev"] = {"ingenic_isp": "isp", "ingenic_t20_isp": "apical",
                "ingenic_t30_ncu": "ncu"}.get(PREFIX.match(ev)[1], "sensor")
    if ev in ACCESS_EVENTS:
        m = ACCESS.match(args)
        if m:
            e["dev"], e["op"] = ACCESS_EVENTS[ev]
            e["off"], e["val"] = int(m[1], 16), int(m[3], 16)
            e["pc"] = int(m[4], 16) if m[4] else None
            e["name"] = m[5]
        return
    if ev in ("image_sensor_write", "image_sensor_read"):
        m = SENSOR.match(args)
        if m:
            e["dev"] = f"sensor{m[1]}"
            e["op"] = "write" if m[3] == "<-" else "read"
            e["off"], e["val"], e["name"] = int(m[2], 16), int(m[4], 16), ""
        return
    if ev == "ingenic_isp_frame":
        m = FRAME.match(args)
        if m:
            e["dev"], e["vic"], e["frame"] = "isp", int(m[1]), int(m[2])
        return
    if ev == "ingenic_t20_isp_frame":
        m = APICAL_FRAME.match(args)
        if m and m[2] == "starts":
            e["dev"], e["vic"], e["frame"] = "apical", 0, int(m[1])
        return
    if ev == "ingenic_isp_vic":
        m = VIC.match(args)
        if m:
            e["vic"], e["start"] = int(m[1]), m[2] == "start"
        return
    if ev == "ingenic_t20_isp_vic":
        m = APICAL_VIC.match(args)
        if m:
            e["vic"], e["start"] = 0, m[1] == "start"
        return
    if ev == "image_sensor_exposure":
        m = EXPOSURE.match(args)
        if m:
            e["dev"] = f"sensor{m[1]}"
            e["lines"], e["gain"] = int(m[2]), int(m[3])
        return
    if ev == "image_sensor_stream":
        m = STREAM.match(args)
        if m:
            e["dev"], e["on"] = f"sensor{m[1]}", m[2] == "1"


def label(dev, off, name):
    width = 4 if dev.startswith("sensor") else 5
    return f"{dev} 0x{off:0{width}x} {name}".rstrip()


def hexval(dev, v):
    return f"0x{v:02x}" if dev.startswith("sensor") else f"0x{v:08x}"


class State:
    """What was last written to each register, and where the frames are"""

    def __init__(self):
        self.regs = {}          # (dev, off) -> dict(val, name, how, frame)
        self.frame = {}         # (dev, vic) -> its last frame number
        self.exposure = {}      # sensor -> (lines16, gain)
        self.streaming = {}     # sensor -> bool
        self.started = False    # a VIC has started
        self.start_frame = None

    def now(self):
        """The frame being programmed: VIC 0's last, or any's"""
        for key in (("isp", 0), ("apical", 0)):
            if key in self.frame:
                return self.frame[key]
        return max(self.frame.values(), default=0)

    def apply(self, e):
        """The event's effect; for a register write, its (old, new) value"""
        op = e.get("op")
        if op in ("write", "shadow", "list"):
            key = (e["dev"], e["off"])
            old = self.regs.get(key)
            self.regs[key] = {"val": e["val"], "name": e["name"],
                              "how": "mmio" if op == "write" else op,
                              "frame": self.now(), "seq": e["seq"],
                              "pc": e.get("pc")}
            return old["val"] if old else None, e["val"]
        if "frame" in e:
            self.frame[(e["dev"], e["vic"])] = e["frame"]
        elif "start" in e:
            if e["start"] and not self.started:
                self.start_frame = self.now()
            self.started |= e["start"]
        elif "lines" in e:
            self.exposure[e["dev"]] = (e["lines"], e["gain"])
        elif "on" in e:
            self.streaming[e["dev"]] = e["on"]
        return None

    def table(self, everything=False, match=None):
        rows = []
        for (dev, off), r in sorted(self.regs.items()):
            if not everything and VOLATILE.search(r["name"]):
                continue
            if match and not (match.search(r["name"]) or
                              match.search(f"0x{off:05x}")):
                continue
            rows.append({"dev": dev, "off": off, **r})
        return rows


def point(spec):
    """--at: "stream" (a VIC starts), "frame:N" (VIC 0's frame N starts,
    before it), "frame:N:V" (VIC V's) or "end" """
    if spec in ("stream", "end"):
        return (spec,)
    m = re.fullmatch(r"frame:(\d+)(?::(\d+))?", spec)
    if not m:
        raise argparse.ArgumentTypeError(f"not a point: {spec}")
    return ("frame", int(m[1]), int(m[2] or 0))


def snapshot(path, at):
    """The state at @at, and whether the trace reached it"""
    s = State()
    for e in events(path):
        if at[0] == "stream" and e.get("start"):
            return s, True
        if at[0] == "frame" and e.get("frame") == at[1] and \
                e.get("vic") == at[2]:
            return s, True
        s.apply(e)
    return s, at[0] == "end"


def describe(at):
    return {"stream": "as a VIC starts", "end": "at the end"}.get(
        at[0], f"as VIC {at[2] if len(at) > 2 else 0}'s frame "
               f"{at[1] if len(at) > 1 else ''} starts")


def cmd_log(a):
    """Each frame's events as they come, then the registers it left changed:
    one line a register, from the value before the frame to the last"""
    match = re.compile(a.match) if a.match else None
    s = State()
    shown = None
    pending = {}                # (dev, off) -> [before, last, how, name]
    out = sys.stdout

    def head():
        nonlocal shown
        f = s.now()
        if f != shown:
            shown = f
            started = "" if s.started else ", before the VICs start"
            out.write(f"frame {f}{started}\n" if f else
                      f"before the first frame{started}\n")

    def change(dev, off, name, old, new, how, pc):
        was = "" if old is None else f"{hexval(dev, old)} -> "
        how = "" if how == "write" else f"  ({how})"
        at = f"  at 0x{pc:08x}" if pc and a.pc else ""
        out.write(f"  {label(dev, off, name)} {was}{hexval(dev, new)}"
                  f"{how}{at}\n")

    def flush():
        for (dev, off), (old, new, how, name, pc) in pending.items():
            if old != new:
                head()
                change(dev, off, name, old, new, how, pc)
        pending.clear()

    for e in events(a.trace, follow=a.follow):
        ev = e["ev"]
        if "frame" in e:
            flush()
            s.apply(e)
            continue
        if a.dev and e["dev"].rstrip("0123456789") not in a.dev:
            s.apply(e)
            continue
        if "op" in e and e["op"] in ("read", "raise"):
            if (e["op"] == "read" and a.reads) or \
                    (e["op"] == "raise" and a.irq):
                head()
                at = f"  at 0x{e['pc']:08x}" if e.get("pc") and a.pc else ""
                out.write(f"  {label(e['dev'], e['off'], e['name'])} "
                          f"{'->' if e['op'] == 'read' else '|='} "
                          f"{hexval(e['dev'], e['val'])}{at}\n")
            continue
        if "op" in e:
            old, new = s.apply(e)
            if (VOLATILE.search(e["name"]) and not a.all) or \
                    (match and not (match.search(e["name"]) or
                                    match.search(f"0x{e['off']:05x}"))):
                continue
            if a.writes:
                head()
                change(e["dev"], e["off"], e["name"], old, new, e["op"],
                       e.get("pc"))
                continue
            key = (e["dev"], e["off"])
            if key in pending:
                pending[key][1:3] = [new, e["op"]]
                pending[key][4] = e.get("pc")
            else:
                pending[key] = [old, new, e["op"], e["name"], e.get("pc")]
            continue
        s.apply(e)
        if NOTES.search(ev) or "lines" in e or \
                (a.channels and CHANNEL.search(ev)) or \
                (a.irq and IRQ.search(ev)):
            head()
            out.write(f"  * {PREFIX.sub('', ev)}: {e['args']}\n")
    flush()
    return 0


def cmd_snapshot(a):
    s, reached = snapshot(a.trace, a.at)
    if not reached:
        print(f"{a.trace}: the trace never gets to that point",
              file=sys.stderr)
        return 1
    rows = s.table(a.all, re.compile(a.match) if a.match else None)
    if a.json:
        json.dump({"at": describe(a.at), "frame": s.now(), "registers": rows,
                   "exposure": s.exposure, "streaming": s.streaming},
                  sys.stdout, indent=1)
        print()
        return 0
    print(f"{a.trace}, {describe(a.at)}: {len(rows)} registers")
    for r in rows:
        pc = f", at 0x{r['pc']:08x}" if r["pc"] else ""
        print(f"  {label(r['dev'], r['off'], r['name']):34} "
              f"{hexval(r['dev'], r['val'])}  {r['how']}, frame "
              f"{r['frame']}{pc}")
    for dev, (lines, gain) in sorted(s.exposure.items()):
        print(f"  {dev} exposure {lines / 16:g} lines, gain {gain / 1024:g}x")
    return 0


def buffer(v):
    """Whether a value looks like a buffer's physical address: one that
    differs between any two runs"""
    return v is not None and not v & 0xff and 0x100000 <= v < 0x20000000


def cmd_diff(a):
    match = re.compile(a.match) if a.match else None
    ignore = re.compile(a.ignore) if a.ignore else None
    sides = []
    for path in (a.a, a.b):
        s, reached = snapshot(path, a.at)
        if not reached:
            print(f"{path}: the trace never gets to that point",
                  file=sys.stderr)
            return 1
        sides.append({(r["dev"], r["off"]): r for r in s.table(a.all, match)
                      if not (ignore and (ignore.search(r["name"]) or
                              ignore.search(f"0x{r['off']:05x}")))})
    ra, rb = sides
    keys = sorted(set(ra) | set(rb))
    diff = [k for k in keys if (ra.get(k) or {}).get("val") !=
            (rb.get(k) or {}).get("val")]
    if a.no_addresses:
        diff = [k for k in diff
                if not (buffer((ra.get(k) or {}).get("val")) and
                        buffer((rb.get(k) or {}).get("val")))]
    if a.json:
        json.dump({"at": describe(a.at), "a": a.a, "b": a.b,
                   "differ": [{"dev": k[0], "off": k[1],
                               "name": (ra.get(k) or rb.get(k))["name"],
                               "a": ra.get(k), "b": rb.get(k)}
                              for k in diff]}, sys.stdout, indent=1)
        print()
        return 1 if diff else 0
    only_a = sum(1 for k in diff if k not in rb)
    only_b = sum(1 for k in diff if k not in ra)
    print(f"{describe(a.at)}: {len(diff)} of {len(keys)} registers differ "
          f"({only_a} set only in A, {only_b} only in B)")
    print(f"  A {a.a}\n  B {a.b}")

    def side(r, dev):
        if not r:
            return f"{'-':10}  {'':16}"
        how = f"{r['how']}, f{r['frame']}"
        return f"{hexval(dev, r['val']):10}  {how:16}"

    for k in diff:
        name = (ra.get(k) or rb.get(k))["name"]
        print(f"  {label(k[0], k[1], name):34} {side(ra.get(k), k[0])}  "
              f"{side(rb.get(k), k[0])}")
    return 1 if diff else 0


def vcd_id(n):
    code = ""
    while True:
        n, r = divmod(n, 94)
        code = chr(33 + r) + code
        if not n:
            return code
        n -= 1


def cmd_vcd(a):
    match = re.compile(a.match) if a.match else None
    changes, counts, names = [], {}, {}
    t0, last_ns = None, 0
    s = State()
    for e in events(a.trace):
        t = e["t"] if e["t"] is not None else e["seq"] * 1e-6
        if t0 is None:
            t0 = t
        ns = last_ns = max(int((t - t0) * 1e9), last_ns)
        if e.get("op") in ("write", "shadow", "list"):
            if VOLATILE.search(e["name"]) and not a.all:
                continue
            if match and not (match.search(e["name"]) or
                              match.search(f"0x{e['off']:05x}")):
                continue
            old, new = s.apply(e)
            if old == new:
                continue
            key = (e["dev"], e["off"])
            names[key] = (f"{e['name']}_{e['off']:05x}" if e["name"]
                          else f"r{e['off']:05x}")
            counts[key] = counts.get(key, 0) + 1
            changes.append((ns, key, new))
        elif "frame" in e:
            s.apply(e)
            key = (e["dev"], f"frame{e['vic']}")
            names[key] = f"vic{e['vic']}_frame"
            counts[key] = 2
            changes.append((ns, key, e["frame"]))
        elif "lines" in e:
            for what, v in (("exposure16", e["lines"]),
                            ("gain1024", e["gain"])):
                key = (e["dev"], what)
                names[key] = what
                counts[key] = 2
                changes.append((ns, key, v))
    # a register written once shows nothing: leave it out unless asked
    keep = [k for k in counts if counts[k] > 1 or a.every]
    ids = {k: vcd_id(i) for i, k in enumerate(keep)}
    with open(a.output, "w") as f:
        f.write("$timescale 1ns $end\n")
        for dev in sorted({k[0] for k in keep}):
            f.write(f"$scope module {dev} $end\n")
            for k in keep:
                if k[0] == dev:
                    n = re.sub(r"[^A-Za-z0-9_]", "_", names[k])
                    f.write(f"$var wire 32 {ids[k]} {n} $end\n")
            f.write("$upscope $end\n")
        f.write("$enddefinitions $end\n")
        last = None
        for ns, k, v in changes:
            if k not in ids:
                continue
            if ns != last:
                f.write(f"#{ns}\n")
                last = ns
            f.write(f"b{v:b} {ids[k]}\n")
    print(f"{a.output}: {len(keep)} signals, {len(changes)} changes")
    return 0


PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "isp-trace.html")


def cmd_serve(a):
    lock = threading.Lock()
    stop = threading.Event()
    s = State()
    live = {"seq": 0, "frames": {}, "notes": [], "exposure": [],
            "changed": {}}

    def follow():
        for e in events(a.trace, follow=True, stop=stop):
            with lock:
                r = s.apply(e)
                if r and r[0] != r[1]:
                    live["changed"][(e["dev"], e["off"])] = s.now()
                if "lines" in e:
                    live["exposure"].append([s.now(), e["dev"], e["lines"],
                                             e["gain"]])
                    del live["exposure"][:-600]
                if NOTES.search(e["ev"]) or "lines" in e:
                    live["notes"].append([s.now(), e["ev"], e["args"]])
                    del live["notes"][:-400]
                live["seq"] = e["seq"]

    def state():
        with lock:
            return {"seq": live["seq"], "frame": s.now(),
                    "started": s.started, "start_frame": s.start_frame,
                    "registers": [dict(r, changed=live["changed"].get(
                        (r["dev"], r["off"]), -1))
                        for r in s.table(a.all)],
                    "exposure": live["exposure"][-300:],
                    "notes": live["notes"][-200:],
                    "streaming": s.streaming}

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
                self.reply(200, json.dumps({"trace": a.trace}))
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

    threading.Thread(target=follow, daemon=True).start()
    httpd = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    httpd.daemon_threads = True
    print(f"http://127.0.0.1:{httpd.server_address[1]}/  ({a.trace}; "
          f"Ctrl-C stops)", flush=True)
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

    def common(sp):
        sp.add_argument("--all", action="store_true",
                        help="the registers that hold no state too")
        sp.add_argument("--match", metavar="RE",
                        help="only registers whose name or offset matches")

    sp = sub.add_parser("log", help="what changed, frame by frame")
    sp.add_argument("trace")
    common(sp)
    sp.add_argument("--writes", action="store_true",
                    help="every write, not only those that change a value")
    sp.add_argument("--reads", action="store_true", help="the reads too")
    sp.add_argument("--irq", action="store_true",
                    help="status bits raised, interrupt lines, Apical events")
    sp.add_argument("--channels", action="store_true",
                    help="channel buffers queued, filled and read")
    sp.add_argument("--pc", action="store_true",
                    help="the guest PC behind each access")
    sp.add_argument("--dev", action="append",
                    choices=["isp", "apical", "ncu", "sensor"],
                    help="only this block's (repeatable)")
    sp.add_argument("-f", "--follow", action="store_true",
                    help="keep reading as the trace grows")
    sp.set_defaults(fn=cmd_log)

    sp = sub.add_parser("snapshot", help="the registers at a point")
    sp.add_argument("trace")
    sp.add_argument("--at", type=point, default=("end",),
                    help="stream, frame:N[:VIC] or end (the default)")
    common(sp)
    sp.add_argument("--json", action="store_true")
    sp.set_defaults(fn=cmd_snapshot)

    sp = sub.add_parser("diff", help="two traces' registers at a point")
    sp.add_argument("a")
    sp.add_argument("b")
    sp.add_argument("--at", type=point, default=("stream",),
                    help="stream (the default), frame:N[:VIC] or end")
    common(sp)
    sp.add_argument("--ignore", metavar="RE",
                    help="leave out registers whose name or offset matches")
    sp.add_argument("--no-addresses", action="store_true",
                    help="leave out pairs of values that both look like "
                         "buffer addresses")
    sp.add_argument("--json", action="store_true")
    sp.set_defaults(fn=cmd_diff)

    sp = sub.add_parser("vcd", help="the registers as waveforms")
    sp.add_argument("trace")
    sp.add_argument("-o", "--output", required=True)
    common(sp)
    sp.add_argument("--every", action="store_true",
                    help="registers written only once too")
    sp.set_defaults(fn=cmd_vcd)

    sp = sub.add_parser("serve", help="a live page, following the trace")
    sp.add_argument("trace")
    sp.add_argument("--port", type=int, default=0)
    sp.add_argument("--interval", type=float, default=0.5,
                    help="seconds between updates (0.5)")
    sp.add_argument("--all", action="store_true",
                    help="the registers that hold no state too")
    sp.set_defaults(fn=cmd_serve)

    a = p.parse_args()
    try:
        return a.fn(a)
    except BrokenPipeError:
        return 0


if __name__ == "__main__":
    sys.exit(main())
