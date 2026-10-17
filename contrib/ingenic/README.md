# Ingenic XBurst1 helper scripts

Companion scripts for booting and driving the Ingenic T-series machine
types (`ingenic-t10` through `ingenic-t33`) provided by this QEMU fork.
Distributed alongside the `qemu-system-mipsel` binary in release tarballs;
also usable in-tree from the build directory.

## Scripts

| Script           | Purpose |
|------------------|---------|
| `qemu-bsp.sh`    | Main launcher. Auto-detects machine type from `-V <variant>`, sets up SSH forwarding, console socket/TCP, USB storage. |
| `qemu-uboot.sh`  | Start QEMU, wait for U-Boot prompt, send commands, capture output. Useful for scripting flash provisioning. |
| `qemu_drive.py`  | Connect to a UART chardev (UNIX socket or TCP), feed commands from stdin, append output to a log file. Pairs with `qemu-bsp.sh -c` / `-T`. |
| `qemu-boot.exp`  | Expect-based wrapper: wait for U-Boot, interrupt autoboot. |
| `qemu-cmd.exp`   | Expect-based wrapper: send a single U-Boot command, capture response. |

## Quick start

Boot a thingino firmware image on T31:

```bash
./qemu-bsp.sh -i thingino-cam.bin
ssh -p 2230 root@localhost   # password: root
```

Boot a different SoC family by selecting a variant — the machine type is
auto-detected from the `-V` prefix:

```bash
./qemu-bsp.sh -i thingino-t20.bin -V t20x      # ingenic-t20
./qemu-bsp.sh -i thingino-t21.bin -V t21n      # ingenic-t21
./qemu-bsp.sh -i thingino-t23.bin -V t23n      # ingenic-t23
./qemu-bsp.sh -i thingino-t30.bin -V t30x      # ingenic-t30
./qemu-bsp.sh -i thingino-t31.bin -V t31x      # ingenic-t31 (default)
./qemu-bsp.sh -i thingino-t10.bin -V t10l      # ingenic-t10
```

Run `./qemu-bsp.sh -h` for the full option list.

## Supported variants

| Family | Variants |
|--------|----------|
| T10    | t10, t10l |
| T20    | t20n, t20x, t20l |
| T21    | t21n, t21l, t21z |
| T23    | t23n, t23x, t23dl, t23zn |
| T30    | t30n, t30x, t30l, t30a, t30z |
| T31    | t31n, t31x, t31l, t31a, t31zl, t31zx, t31al, t31zc, t31lc |
| T32    | t32nq, t32lq, t32xq, t32zn |
| T33    | t33n |

The variant selects EFUSE/PRid bits, PLL config, and RAM size — read by
the firmware's `soc` tool and libimp.

## Locating the QEMU binary

`qemu-bsp.sh` looks for `qemu-system-mipsel` in this order:

1. `$QEMU` environment variable
2. Same directory as the script (release tarball layout)
3. `../../build/qemu-system-mipsel` (in-tree dev layout)
4. `$PATH`

Set `QEMU=/some/other/path` to override.

## Useful environment overrides

```bash
QEMU=...      # path to qemu-system-mipsel
KERNEL=...    # path to uImage (required in linux mode)
UBOOT=...     # path to u-boot-spl ELF (required in uboot mode)
FLASH=...     # path to flash image (required in linux/uboot mode)
CMDLINE=...   # kernel command line (linux mode only; sensible default)
```

## Console scripting

```bash
# Expose UART1 as a UNIX socket and drive it from Python:
./qemu-bsp.sh -i fw.bin -c /tmp/qemu.sock &
echo "uname -a" | ./qemu_drive.py /tmp/qemu.sock

# Or as TCP for the /uart Claude skill:
./qemu-bsp.sh -i fw.bin -T 3331
```

## SPI NAND (T40, T41, A1)

`-global ingenic-a1-sfc.nand-id=0xc851` puts a GigaDevice GD5F1GQ5UE
(128 MiB) on the XBurst2 flash controller in place of the NOR;
`nand-blocks` sets the size in 128 KiB blocks (default 1024). The image
holds the page data back to back, as thingino's NAND images do. Pad it to
the chip size with 0xff, or what the guest writes past its end is lost
when QEMU exits:

```bash
head -c $(( (128 << 20) - $(stat -c %s fw.bin) )) /dev/zero | tr '\0' '\377' >> fw.bin
qemu-system-mipsel -M ingenic-t41 -m 128M -display none \
    -global ingenic-a1-sfc.nand-id=0xc851 \
    -drive file=fw.bin,format=raw,if=none,id=flash0 \
    -serial null -serial stdio
```

## Timing

Guest time follows the host clock and the CPU clock comes from the CPM,
so U-Boot and Linux delays last as long as on the boards. Code runs at
QEMU's own speed, measured against T21, T31 and T41 silicon: integer code
1.5-3x faster than the chip, FPU code 2-5x slower, code that misses the
guest TLB 7-25x slower (each refill is an exception).

The `ips` plugin caps the instruction rate (it only slows QEMU down):

```bash
qemu-system-mipsel ... -plugin plugins/libips.so,ips=1030m
```

Instructions per CPU clock measured on the silicon:

| Code                     | XBurst1 (T21, T31) | XBurst2 (T41) |
|--------------------------|--------------------|---------------|
| ALU loop                 | 0.90               | 1.31          |
| qsort                    | 0.74               | 0.71          |
| unaligned copy           | 0.52-0.57          | 0.68          |
| dependent floating point | 0.27               | 0.66          |

For typical integer code, `ips` = 0.74 x the CPU clock: T31X (1392 MHz)
1030m, T41NQ (1400 MHz) 1000m, T20X or T21N (about 860 MHz) 640m.

The two cores of the T40, T41 and A1 see each other's memory accesses in
program order, as on the silicon; fencing the host for that costs those
SoCs about 10 % of their speed.

## Cache coherency checker

The XBurst L1 caches are write-back and do not snoop DMA, so a missing
cache flush or invalidate works in QEMU and fails on a board. The
`coherency` plugin follows every L1 line and reports where the guest
relies on coherence the silicon lacks:

```bash
qemu-system-mipsel ... -plugin plugins/libcoherency.so,ram=128M,l1=32K \
    -d plugin -D coherency.log
```

`ram` is the guest RAM size, `l1` the size of each L1: 16K on the T21 and
T23, 32K on the others. A report marked `latent` concerns a line that an
LRU model of the cache had already evicted, so the board most likely got
away with it. The report kinds are described at the top of
`contrib/plugins/coherency.c`. Boots take about 2.5x longer.

## Sample firmwares

The Thingino project publishes firmware images for each supported camera
model: <https://github.com/themactep/thingino-firmware>
