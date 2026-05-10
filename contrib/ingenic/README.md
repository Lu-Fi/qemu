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
KERNEL=...    # path to uImage (linux mode, default /tmp/uImage-uncomp)
UBOOT=...     # path to u-boot-spl ELF (uboot mode)
FLASH=...     # path to flash image (linux/uboot mode)
CMDLINE=...   # kernel command line (linux mode only)
```

## Console scripting

```bash
# Expose UART1 as a UNIX socket and drive it from Python:
./qemu-bsp.sh -i fw.bin -c /tmp/qemu.sock &
echo "uname -a" | ./qemu_drive.py /tmp/qemu.sock

# Or as TCP for the /uart Claude skill:
./qemu-bsp.sh -i fw.bin -T 3331
```

## Sample firmwares

The Thingino project publishes firmware images for each supported camera
model: <https://github.com/themactep/thingino-firmware>

Or grab pre-built test images: see the firmware-thingino-sample directory
in the qemu-project tree.
