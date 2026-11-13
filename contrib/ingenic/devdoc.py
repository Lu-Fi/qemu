#!/usr/bin/env python3
#
# Generate the Ingenic device pages (contrib/ingenic/devices/) from the
# model itself.
#
# What it reads:
#   - the machine names (DEFINE_MACHINE in hw/mips/ingenic-*-board.c) and
#     the variant tables of hw/mips/ingenic-*-soc.c
#   - the census table tests/qtest/ingenic-census-regs.inc
#   - the built qemu-system-mipsel: for each machine and variant, "info
#     qtree" (every device on the system bus: its QOM type, its MMIO
#     windows, the profile of an ingenic-busregs, the name of an
#     unimplemented-device) and "info mtree -f" (the RAM, ROM and filler
#     regions that no device owns). The C tables that place the blocks are
#     full of per-family conditions, so the machine is asked instead of
#     the source being parsed.
#
# Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
#
# SPDX-License-Identifier: GPL-2.0-or-later

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
TREE = os.path.dirname(os.path.dirname(HERE))
OUTDIR = os.path.join(HERE, "devices")

QEMU_TIMEOUT = 60

# busregs profiles that hold no register: they read 0 and drop writes
EMPTY_PROFILES = {"zero4k", "zero64k", "gpio-gap"}

STATES = """\
| State | Meaning |
|---|---|
| modelled | a real device model that the census holds no register of; its own qtest checks it |
| register-accurate | the census holds registers of this block as the part's silicon read them, or it is an ingenic-busregs window with measured reset values and write masks |
| register-less window | an ingenic-busregs window that reads 0 and drops writes, as measured on the part |
| stub | an unimplemented-device: reads 0, drops writes and logs the access (`-d unimp`) |
| helper region | a plain memory region that no device owns (RAM, ROM, bus filler), set up in hw/mips/ingenic-boot.c and the SoC files |
"""


def read(path):
    with open(os.path.join(TREE, path), encoding="utf-8") as f:
        return f.read()


def machines_and_socs():
    """machine name -> SoC QOM type, from the board sources"""
    out = {}
    hw = os.path.join(TREE, "hw/mips")
    for fn in sorted(os.listdir(hw)):
        m = re.fullmatch(r"ingenic-(t31|t40|a1)-board\.c", fn)
        if not m:
            continue
        for name in re.findall(r'DEFINE_MACHINE\("(ingenic-[a-z0-9]+)"',
                               read("hw/mips/" + fn)):
            out[name] = "ingenic-" + m.group(1)
    return out


def variants():
    """SoC type -> variant names, from the *_variants[] tables"""
    out = {}
    hw = os.path.join(TREE, "hw/mips")
    for fn in sorted(os.listdir(hw)):
        m = re.fullmatch(r"ingenic-(t31|t40|a1)-soc\.c", fn)
        if not m:
            continue
        src = read("hw/mips/" + fn)
        names = []
        for tbl in re.finditer(r"_variants\[\]\s*=\s*\{(.*?)^\};", src,
                               re.S | re.M):
            names += re.findall(r'^\s*\{\s*"([a-z0-9]+)"', tbl.group(1), re.M)
        out["ingenic-" + m.group(1)] = names
    return out


def variant_machine(var, machines):
    """The machine a variant belongs to, by the part number in its name"""
    m = re.match(r"([ta])(\d+)", var)
    if var.startswith("cw080"):
        cand = "ingenic-t32"
    elif var.startswith("c100"):
        cand = "ingenic-t31"
    elif m:
        cand = "ingenic-%s%s" % (m.group(1), m.group(2))
    else:
        return None
    return cand if cand in machines else None


def census():
    """[(part, machine, soc, variant, [addresses])] from the .inc"""
    src = read("tests/qtest/ingenic-census-regs.inc")
    regs = {}
    for m in re.finditer(r"static const CensusReg (\w+)_regs\[\] = \{(.*?)\};",
                         src, re.S):
        regs[m.group(1)] = [int(a, 16) for a in
                            re.findall(r"\{\s*(0x[0-9a-f]+),", m.group(2))]
    parts = []
    for m in re.finditer(r'\{\s*"(\w+)",\s*"(ingenic-\w+)",\s*"(ingenic-\w+)",'
                         r'\s*"(\w+)",\s*(\w+)_regs,', src):
        parts.append((m.group(1), m.group(2), m.group(3), m.group(4),
                      regs[m.group(5)]))
    return parts


def run_qemu(qemu, machine, soc, variant, tmp):
    """(qtree text, mtree text) of one machine and variant

    QMP on stdio with human-monitor-command, never an interactive monitor;
    the guest is never started. The whole process group is killed when the
    deadline passes.
    """
    # some boards refuse to start with nothing to boot, so a blank kernel
    # image stands in; it is never run
    kern = os.path.join(tmp, "blank.bin")
    cmd = [qemu, "-M", machine, "-S", "-display", "none", "-nodefaults",
           "-qmp", "stdio", "-kernel", kern]
    if variant:
        cmd += ["-global", "%s.soc-variant=%s" % (soc, variant)]

    def hmp(c):
        return json.dumps({"execute": "human-monitor-command",
                           "arguments": {"command-line": c}})

    inp = "\n".join([json.dumps({"execute": "qmp_capabilities"}),
                     hmp("info qtree"), hmp("info mtree -f"),
                     json.dumps({"execute": "quit"})]) + "\n"
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, text=True,
                         start_new_session=True)
    try:
        out, err = p.communicate(inp, timeout=QEMU_TIMEOUT)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.communicate()
        sys.exit("%s %s: timed out after %ds" %
                 (machine, variant, QEMU_TIMEOUT))
    finally:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    rets = []
    for line in out.splitlines():
        try:
            j = json.loads(line)
        except ValueError:
            continue
        if isinstance(j.get("return"), str):
            rets.append(j["return"])
    if len(rets) != 2:
        sys.exit("%s %s: no monitor output: %s" %
                 (machine, variant, err.strip()))
    return rets


def parse_qtree(text):
    """system-bus devices: [{type, props, mmio: [(addr, size)]}]"""
    devs = []
    cur = None
    for line in text.splitlines():
        m = re.match(r"^  dev: ([^,]+), id", line)
        if m:
            cur = {"type": m.group(1), "props": {}, "mmio": []}
            devs.append(cur)
            continue
        if not line.startswith("    "):
            cur = None
            continue
        if cur is None or line.startswith("     "):
            continue
        m = re.match(r"^    mmio ([0-9a-f]+)/([0-9a-f]+)", line)
        if m:
            cur["mmio"].append((int(m.group(1), 16), int(m.group(2), 16)))
            continue
        m = re.match(r'^    ([\w-]+) = "?([^"]*)"?', line)
        if m:
            cur["props"][m.group(1)] = m.group(2)
    return devs


def parse_mtree(text):
    """the flat view of address space "memory": [(start, end, prio, kind, name)]

    The view numbers differ between runs, so the view is picked by its
    address space name.
    """
    out = []
    started = False
    for line in text.splitlines():
        if line.startswith("FlatView"):
            started = False
            continue
        if line.strip().startswith('AS "memory"'):
            started = True
            continue
        m = re.match(r"^\s+([0-9a-f]+)-([0-9a-f]+) \(prio (-?\d+), "
                     r"([\w-]+)\): (\S+)", line)
        if started and m:
            out.append((int(m.group(1), 16), int(m.group(2), 16),
                        int(m.group(3)), m.group(4), m.group(5)))
    return out


def blocks(qtree, mtree):
    """rows: dicts with addr, size, name, type, kind"""
    rows = []
    for d in qtree:
        for i, (a, s) in enumerate(d["mmio"]):
            r = {"addr": a, "size": s, "type": d["type"], "name": d["type"]}
            if d["type"] == "unimplemented-device":
                r["kind"] = "stub"
                r["name"] = d["props"].get("name", "?")
            elif d["type"] == "ingenic-busregs":
                model = d["props"].get("model", "?")
                r["kind"] = "empty" if model in EMPTY_PROFILES else "busregs"
                r["name"] = "busregs " + model
            else:
                r["kind"] = "device"
                if len(d["mmio"]) > 1:
                    r["name"] = "%s window %d" % (d["type"], i)
            rows.append(r)
    # a stub that a real block covers answers nothing: leave it out
    live = [(r["addr"], r["addr"] + r["size"]) for r in rows
            if r["kind"] != "stub"]
    rows = [r for r in rows if r["kind"] != "stub" or not any(
        lo <= r["addr"] and r["addr"] + r["size"] <= hi for lo, hi in live)]
    # regions no device owns: RAM, ROM and the bus fillers
    owned = [(r["addr"], r["addr"] + r["size"] - 1) for r in rows]
    extra = []
    for a, b, prio, kind, name in mtree:
        if prio <= -1000:
            continue
        if any(lo <= a and b <= hi for lo, hi in owned):
            continue
        if extra and extra[-1]["name"] == name and \
                extra[-1]["end"] + 1 == a:
            extra[-1]["end"] = b
            continue
        extra.append({"addr": a, "end": b, "name": name, "mkind": kind})
    for e in extra:
        rows.append({"addr": e["addr"], "size": e["end"] - e["addr"] + 1,
                     "type": "memory region (%s)" % e["mkind"],
                     "name": e["name"], "kind": "helper"})
    rows.sort(key=lambda r: (r["addr"], -r["size"], r["name"]))
    # number the instances that share a name, in address order
    byname = {}
    for r in rows:
        byname.setdefault(r["name"], []).append(r)
    for name, rs in byname.items():
        if len(rs) > 1:
            for i, r in enumerate(rs):
                r["name"] = "%s #%d" % (name, i)
    return rows


def census_counts(rows, addrs):
    """block index -> census registers, each in the smallest block holding it"""
    counts = {}
    for a in addrs:
        best = None
        for i, r in enumerate(rows):
            if r["kind"] == "helper":
                continue
            if r["addr"] <= a < r["addr"] + r["size"]:
                if best is None or r["size"] < rows[best]["size"]:
                    best = i
        if best is not None:
            counts[best] = counts.get(best, 0) + 1
    return counts


def state(r, n):
    if r["kind"] == "stub":
        return "stub"
    if r["kind"] == "empty":
        return "register-less window"
    if r["kind"] == "busregs":
        return "register-accurate"
    if r["kind"] == "helper":
        return "helper region"
    return "register-accurate" if n else "modelled"


def hexs(n):
    return "0x%08x" % n


def size_s(n):
    for unit, sh in (("MiB", 20), ("KiB", 10)):
        if n >= 1 << sh and n % (1 << sh) == 0:
            return "%d %s" % (n >> sh, unit)
    return "0x%x" % n


def key(r):
    return (r["addr"], r["size"], r["name"], r["type"])


def page(machine, soc, ref, runs, parts):
    """one machine's Markdown page"""
    rows = runs[ref]
    mparts = [p for p in parts if p[1] == machine]
    cnt = {p[0]: census_counts(rows, p[4]) for p in mparts}
    o = ["# %s\n" % machine,
         "<!-- Generated by contrib/ingenic/devdoc.py from the model and "
         "the census table: do not edit. -->\n",
         "Memory map of the `%s` machine, SoC type `%s`, with the `%s` "
         "variant and the default `-m`, as the machine lists it (`info "
         "qtree` and `info mtree -f` of the built binary). The reference "
         "variant is the first one with a census part, else the machine's "
         "default.\n" % (machine, soc, ref or "default"),
         "## Blocks\n",
         "| Base | Size | Block | QEMU type | State | Census |",
         "|---|---|---|---|---|---|"]
    totals = {}
    for i, r in enumerate(rows):
        n = sum(c.get(i, 0) for c in cnt.values())
        st = state(r, n)
        totals[st] = totals.get(st, 0) + 1
        cs = ", ".join("%s %d" % (p, c[i]) for p, c in cnt.items()
                       if i in c)
        o.append("| %s | %s | %s | `%s` | %s | %s |" %
                 (hexs(r["addr"]), size_s(r["size"]), r["name"], r["type"],
                  st, cs))
    o.append("")
    o.append("%d blocks: %s.\n" % (len(rows), ", ".join(
        "%d %s" % (v, k) for k, v in sorted(totals.items()))))
    o.append("A block is listed with the span its device maps. A stub "
             "wholly covered by a model is not listed. A stub or a "
             "helper region has the lowest priority, so any block inside "
             "its span answers in its place. A census register counts for "
             "the smallest block that holds it; the Census column gives, "
             "per census part, how many registers of the table are in the "
             "block.\n")
    o.append("## Census parts\n")
    if mparts:
        o.append("| Part | Variant | Registers | In a block of this map |")
        o.append("|---|---|---|---|")
        for p in mparts:
            o.append("| %s | %s | %d | %d |" %
                     (p[0], p[3], len(p[4]), sum(cnt[p[0]].values())))
        o.append("")
        o.append("The registers are counted against the reference "
                 "variant's map, so a part of another variant may have "
                 "some outside it.\n")
    else:
        o.append("None: no silicon census has been taken of this machine, "
                 "so none of its blocks is marked register-accurate "
                 "from the census.\n")
    base = {key(r) for r in rows}
    groups = {}
    for v, vr in runs.items():
        if v == ref:
            continue
        cur = {key(r) for r in vr}
        groups.setdefault((frozenset(cur - base), frozenset(base - cur)),
                          []).append(v)
    o.append("## Variants whose map differs\n")
    groups = {k: v for k, v in groups.items() if k[0] or k[1]}
    if not groups:
        o.append("None: every variant of this machine maps the same "
                 "blocks.\n")
    for (add, rem), vs in sorted(groups.items(),
                                 key=lambda kv: sorted(v or "" for v in kv[1])):
        o.append("`%s`:\n" % "`, `".join(sorted(v or "default" for v in vs)))
        for k in sorted(rem):
            o.append("- without %s %s (`%s`)" % (hexs(k[0]), k[2], k[3]))
        for k in sorted(add):
            o.append("- adds %s %s, %s (`%s`)" %
                     (hexs(k[0]), size_s(k[1]), k[2], k[3]))
        o.append("")
    return "\n".join(o).rstrip() + "\n"


def index(meta, parts):
    o = ["# Ingenic device pages: machines\n",
         "<!-- Generated by contrib/ingenic/devdoc.py: do not edit. -->\n",
         "One page per machine, generated from the model. The boot paths "
         "and how the models were checked against silicon are in "
         "[README.md](README.md).\n",
         "| Machine | SoC type | Reference variant | Blocks | Census parts |",
         "|---|---|---|---|---|"]
    for m in sorted(meta):
        soc, ref, n = meta[m]
        mp = ", ".join(p[0] for p in parts if p[1] == m) or "none"
        o.append("| [%s](%s.md) | `%s` | %s | %d | %s |" %
                 (m, m, soc, ref, n, mp))
    o.append("")
    o.append("## States\n")
    o.append(STATES)
    return "\n".join(o)


def generate(qemu):
    mach = machines_and_socs()
    parts = census()
    jobs = [(m, soc, None) for m, soc in mach.items()]
    for soc, names in variants().items():
        for v in names:
            m = variant_machine(v, mach)
            if m and mach[m] == soc:
                jobs.append((m, soc, v))
    with tempfile.TemporaryDirectory() as tmp:
        with open(os.path.join(tmp, "blank.bin"), "wb") as f:
            f.write(bytes(4096))
        with ThreadPoolExecutor(max_workers=4) as ex:
            res = list(ex.map(lambda j: (j, run_qemu(qemu, *j, tmp)), jobs))
    runs = {}
    for (m, soc, v), (qt, mt) in res:
        runs.setdefault(m, {})[v] = blocks(parse_qtree(qt), parse_mtree(mt))
    out = {}
    meta = {}
    for m, soc in mach.items():
        cp = [p for p in parts if p[1] == m and p[3] in runs[m]]
        ref = cp[0][3] if cp else None
        out[m + ".md"] = page(m, soc, ref, runs[m], parts)
        meta[m] = (soc, ref or "default", len(runs[m][ref]))
    out["index.md"] = index(meta, parts)
    return out


def main():
    ap = argparse.ArgumentParser(description="Generate the Ingenic device "
                                 "pages from the model")
    ap.add_argument("--qemu", default=os.path.join(TREE, "build",
                                                   "qemu-system-mipsel"),
                    help="the built qemu-system-mipsel")
    ap.add_argument("--check", action="store_true",
                    help="write nothing; fail if a page would change")
    args = ap.parse_args()

    pages = generate(args.qemu)
    os.makedirs(OUTDIR, exist_ok=True)
    stale = []
    for name, text in sorted(pages.items()):
        path = os.path.join(OUTDIR, name)
        old = None
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                old = f.read()
        if old != text:
            stale.append(name)
            if not args.check:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(text)
    # a page of a machine that is gone
    if os.path.isdir(OUTDIR):
        for name in sorted(os.listdir(OUTDIR)):
            if name.endswith(".md") and name != "README.md" and \
                    name not in pages:
                stale.append(name)
                if not args.check:
                    os.remove(os.path.join(OUTDIR, name))
    if args.check and stale:
        sys.exit("contrib/ingenic/devices is out of date (%s): run "
                 "contrib/ingenic/devdoc.py and commit the result" %
                 ", ".join(stale))
    for name in stale:
        print("wrote", name)


if __name__ == "__main__":
    main()
