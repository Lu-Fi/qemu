#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""
Write tests/qtest/ingenic-ldc-corpus-data.inc from a T30X's LDC probe runs.

usage: ldc-corpus.py MISSES PROBEDIR CAPDIR... > ingenic-ldc-corpus-data.inc

PROBEDIR holds the hang sets hN.sh / hN.out: one job per line of the .sh
(the probe tool's register writes) and, on the same line of the .out, what
the part did (ended, or TIMEOUT). Only the final register values of a job
are kept, as the change from the job before, the outcome as a bit. Stream,
per job: byte 0 = hang << 7 | fast << 6 | n changes, then n times a
register index (into the table of registers) and the value as a LEB128.
The fast jobs are the
ones next to a change of outcome in their set and every 32nd (every 128th on
1280 x 720, which the model computes in full), the subset the test runs
unless told to run all.

Each CAPDIR holds runs.log, one probe tool run a line (the tag, the input of
an earlier job and of the job, key=value settings, then the part's reply),
and TAG.yuv, the part's whole output picture (a partial fetch is
skipped): the job's registers and inputs are kept with
the CRC32 of that output. A job captured twice with two different CRCs is
dropped.

MISSES lists what is left out, one "set index label reason" a line: the
hang sets' jobs by their number in the set (from 0), the captures as
"cap - TAG reason".
"""
import base64
import glob
import os
import sys
import textwrap
import zlib

SETS = []
SKIP = (0, 4)
LICENSE = 'GPL-2.0-or-' + 'later'
LUT = 16
DEF = dict(w=1280, h=720, k1x=0, k2x=0, k1y=0, k2y=0, px=2048, py=2048,
           r2=1991, udis=0, wr=12, wrs=1, rd=15, rds=15, yf=255, uf=128,
           vf=128, ctrl=0x38, ylut='', uvlut='', of='Z', wrA='', wrsA='',
           fetch=1)


def jobs(d, name):
    sh = [x for x in open(f'{d}/{name}.sh').read().splitlines()
          if x.startswith('echo')]
    out = [x.split() for x in open(f'{d}/{name}.out') if len(x.split()) > 4]
    assert len(sh) == len(out), name
    for n, (line, o) in enumerate(zip(sh, out)):
        t = line.split('/tmp/ldcrun 130a0000 ')[1].split(' | ')[0].split()
        regs = {}
        i = 0
        while i < len(t):
            if t[i] == 'w':
                off, val = int(t[i + 1], 16), int(t[i + 2], 16)
                if off == 0 and val == 0x39:
                    break
                if off not in SKIP:
                    regs[off] = val
                i += 3
            elif t[i] == 'f':
                i += 4
            else:
                raise SystemExit('token ' + t[i])
        yield n, o[0], regs, o[4] != 'ok'


def leb(v):
    b = bytearray()
    while True:
        c = v & 0x7f
        v >>= 7
        b.append(c | (0x80 if v else 0))
        if not v:
            return b


def num(x):
    return int(x, 0) if isinstance(x, str) else x


def spec(s):
    """an input as (kind, arg): Z zeroes, N xorshift noise of a hex seed,
    B a byte-index bit as stripes; None for what the corpus cannot make"""
    if s == 'Z':
        return ('Z', 0)
    if s[0] == 'N':
        return ('N', int(s[1:] or '1234', 16))
    if s[0] == 'B':
        return ('B', int(s[1:]))
    return None


def captures(dirs):
    caps = {}
    for d in dirs:
        for line in open(f'{d}/runs.log'):
            left, _, res = line.partition(' :: ')
            t = left.split()
            tag, a, b = t[:3]
            a_ = dict(DEF)
            for kv in t[3:]:
                k, _, v = kv.partition('=')
                a_[k] = v
            sa = None if a == '-' else spec(a)
            sb = spec(b) if b != '-' else None
            if ('TIMEOUT' in res or num(a_['fetch']) == 0 or sb is None or
                    (a != '-' and sa is None) or a_['of'][0] not in 'ZKFN' or
                    not os.path.exists(f'{d}/{tag}.yuv')):
                continue
            w, h = num(a_['w']), num(a_['h'])
            yuv = open(f'{d}/{tag}.yuv', 'rb').read()
            if len(yuv) != w * h * 3 // 2:
                continue
            lut = {}
            for key in ('ylut', 'uvlut'):
                v = [num(x) if x else 0 for x in a_[key].split(',')]
                assert len(v) <= LUT
                lut[key] = v + [0] * (LUT - len(v))
            wr = (num(a_['wr']) | num(a_['wrs']) << 8 | num(a_['rd']) << 16 |
                  num(a_['rds']) << 24)
            wra = 0
            if a_['wrA'] != '':
                wra = (num(a_['wrA']) | num(a_['wrsA'] or a_['wrs']) << 8 |
                       num(a_['rd']) << 16 | num(a_['rds']) << 24)
            of = a_['of']
            if of[0] == 'N':
                ofs = ('N', int(of[1:], 16))
            else:
                ofs = (of[0], 0)
            job = dict(w=w, h=h,
                       r=[num(a_['k1x']) | num(a_['k2x']) << 16,
                          num(a_['k1y']) | num(a_['k2y']) << 16,
                          num(a_['udis']), wr,
                          num(a_['px']) | num(a_['py']) << 16,
                          num(a_['r2']),
                          num(a_['vf']) | num(a_['uf']) << 8 |
                          num(a_['yf']) << 16],
                       ctrl=num(a_['ctrl']), wra=wra, a=sa, b=sb, of=ofs,
                       ylut=lut['ylut'], uvlut=lut['uvlut'])
            key = repr(job)
            caps.setdefault(key, []).append((tag, job, zlib.crc32(yuv)))
    out = []
    for runs in caps.values():
        if len({c for _, _, c in runs}) == 1:
            out.append(runs[0])
    return out


def main():
    miss = {}
    for x in open(sys.argv[1]):
        if x.strip() and not x.startswith('#'):
            s, n, lab, why = x.strip().split(None, 3)
            miss[(s, int(n) if n != '-' else lab)] = (lab, why)
    d = sys.argv[2]
    for f in sorted(glob.glob(f'{d}/*.sh')):
        s = os.path.basename(f)[:-3]
        try:
            list(jobs(d, s))
        except (AssertionError, FileNotFoundError):
            continue
        if s != 'h0':
            SETS.append(s)
    def key(r):
        return tuple(sorted((a, b) for a, b in r.items() if b))

    bad = {key(r) for s in SETS for n, _, r, _ in jobs(d, s)
           if (s, n) in miss}
    first = {}
    keep = []
    for s in SETS:
        for n, lab, r, h in jobs(d, s):
            k = key(r)
            if k in bad:
                if (s, n) not in miss:
                    miss[(s, n)] = (lab, 'the same registers as a job left '
                                    'out')
            elif k not in first:
                first[k] = (s, n, lab, r, h)
            elif first[k][4] != h:
                for t in (first[k], (s, n, lab, r, h)):
                    miss[(t[0], t[1])] = (t[2], 'the same registers ended '
                                          'and hung the part')
                bad.add(k)
    keep = [j for j in first.values() if key(j[3]) not in bad]
    caps = [c for c in captures(sys.argv[3:]) if ('cap', c[0]) not in miss]
    regs = sorted({k for j in keep for k in j[3]})
    ix = {r: i for i, r in enumerate(regs)}
    stream = bytearray()
    prev = {}
    groups = []
    fast = set()
    for s in SETS:
        ks = [j for j in keep if j[0] == s]
        for k, j in enumerate(ks):
            big = j[3][0x8] == 720 << 16 | 1280
            if (k % (128 if big else 32) == 0 or
                    (k and ks[k - 1][4] != j[4]) or
                    (k + 1 < len(ks) and ks[k + 1][4] != j[4])):
                fast.add((s, j[1]))
    nfast = len(fast)
    for n, i, _, r, h in keep:
        ch = [(k, r.get(k, 0)) for k in regs if r.get(k, 0) != prev.get(k, 0)]
        assert len(ch) < 64
        stream.append(h << 7 | ((n, i) in fast) << 6 | len(ch))
        for k, v in ch:
            stream.append(ix[k])
            stream += leb(v)
        prev = r
        if not groups or groups[-1][0] != n:
            groups.append([n, 0])
        groups[-1][1] += 1
    nhang = sum(1 for j in keep if j[4])
    head = ("A T30X's LDC jobs for ingenic-ldc-corpus-test.c, synthetic "
            "pictures only. %d jobs and whether each ended or hung the part "
            "(WORK still set after the time out), as the registers the probe "
            "tool wrote and the part's answer: %d ended, %d hung, %d of them "
            "the default subset. And %d jobs on noise pictures with the "
            "CRC32 of the part's output. Written by "
            "contrib/ingenic/ldc-corpus.py; do not edit by hand."
            % (len(keep), len(keep) - nhang, nhang, nfast, len(caps)))
    left = ("Left out: jobs the part hung on that look random (the model "
            "ends them), the sweeps and captures the model does not follow "
            "yet, and jobs repeated in two sets with two outcomes:")
    print('/*')
    for t in (head, left):
        print(textwrap.fill(t, 76, initial_indent=' * ',
                            subsequent_indent=' * '))
        print(' *')
    cnt = {}
    for (s, n), (lab, why) in sorted(miss.items(), key=lambda m: str(m)):
        cnt.setdefault((s, why), []).append(lab)
    for (s, why), labs in sorted(cnt.items()):
        if len(labs) > 12:
            labs = labs[:8] + ['...']
        print(textwrap.fill('%s, %s (%d): %s' % (s, why, len(cnt[(s, why)]),
                                                 ' '.join(labs)), 76,
                            initial_indent=' *   ',
                            subsequent_indent=' *       '))
    print(' * SPDX' + '-License-Identifier: ' + LICENSE)
    print(' */\n')
    print('static const uint16_t corpus_regs[] = {')
    for i in range(0, len(regs), 8):
        print('    ' + ' '.join('0x%03x,' % r for r in regs[i:i + 8]))
    print('};\n')
    print('static const struct {\n    const char *name;\n    int n;\n}'
          ' corpus_sets[] = {')
    for n, c in groups:
        print('    { "%s", %d },' % (n, c))
    print('};\n')
    b = base64.b64encode(bytes(stream)).decode()
    print('static const char corpus_stream[] =')
    for i in range(0, len(b), 76):
        print('    "%s"' % b[i:i + 76])
    print('    ;\n')
    print('''typedef struct {
    char kind;                  /* Z zeroes, N noise, B stripes, K kept */
    uint32_t arg;
} CorpusFill;

typedef struct {
    const char *tag;
    uint16_t w, h;
    uint32_t regs[7];           /* 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24 */
    uint32_t wr_a;              /* 0x18 for the first job, 0 = the same */
    CorpusFill a, b, out;       /* first job's input, the job's, its output */
    uint16_t ylut[%d], uvlut[%d];
    uint32_t crc;
} CorpusJob;

static const CorpusJob corpus_jobs[] = {''' % (LUT, LUT))

    def fill(f):
        return "{ '%s', 0x%x }" % f if f else '{ 0, 0 }'

    def lut(v):
        while v and not v[-1]:
            v = v[:-1]
        return '{ %s }' % ', '.join(str(x) for x in v) if v else '{ 0 }'
    for tag, j, crc in sorted(caps, key=lambda c: c[0]):
        print('    { "%s", %d, %d,\n      { %s }, 0x%x,\n      %s, %s, %s,\n'
              '      %s,\n      %s,\n      0x%08x },' %
              (tag, j['w'], j['h'], ', '.join('0x%x' % x for x in j['r']),
               j['wra'], fill(j['a']), fill(j['b']), fill(j['of']),
               lut(j['ylut']), lut(j['uvlut']), crc))
    print('};')


main()
