# Ingenic devices: boot paths and how the models were checked

The per-machine pages next to this file are generated from the model by
`contrib/ingenic/devdoc.py` (see [index.md](index.md)); this page is
written by hand. Machines come in three SoC families, each built from one
SoC type:

| Family | Machines | SoC type |
|---|---|---|
| XBurst1 | `ingenic-t10`, `t20`, `t21`, `t23`, `t30`, `t31`, `t32`, `t33` | `ingenic-t31` |
| XBurst2 | `ingenic-t40`, `ingenic-t41` | `ingenic-t40` |
| A1 | `ingenic-a1` | `ingenic-a1` |

A variant (`-global <soc type>.soc-variant=...`, or `-V` in `qemu-bsp.sh`)
selects the part inside a machine; a page lists what each variant changes.

## Boot paths

Three ways in, on every family:

- Flash image, no `-bios`: the machine loads the SPL from the flash image
  (SPI NOR or NAND) itself and starts it as the bootrom leaves the chip
  (clocks and GPIO as the bootrom sets them), then the SPL loads U-Boot
  and the kernel as on the part.
- `-bios <bootrom>`: the CPU starts at 0xbfc00000 in the chip's real
  bootrom (mapped at 0x1fc00000), which picks its boot source from the
  boot-select pads, runs the secure boot when the eFuse enables it, and
  loads the SPL as the chip does. The XBurst1 bootroms repeat or alias
  over NEMC bank 0 as each was measured.
- `-kernel`: a kernel is loaded directly, with no flash image.

The pads (`ingenic-gpio.in-pc`), the eFuse words (`efuse-security`,
`efuse-keyhash`, `efuse-chipid`, `efuse-file`) and the RAM aliases are
described in the "Boot ROM and secure boot" and "Supported variants"
sections of [../README.md](../README.md). `-S -kernel <blank file>` is how
the generator starts a machine without running it.

On the T31, T23, T32, T33, T41 and A1 a RISC-V core (and the T33's TNPU
core) runs the fast-boot firmware that the SPL loads; its boot is in the
"Zeratul fast boot" section of the same README.

## How the models were checked

- Every device has a qtest in `tests/qtest/ingenic-*-test.c`, run by
  `.github/scripts/run-qtests.sh`; a block shown as "modelled" has at
  least its own test and no census entries.
- A "register-accurate" block is one that the silicon census
  (`tests/qtest/ingenic-census-regs.inc`) holds registers of, or an
  `ingenic-busregs` window whose reset values and write masks were
  measured on the part. The census is about 3000 registers read at the
  U-Boot prompt from T10L, T20X, T21N, T23N, T23DL, T30X, T31X, T32LQ,
  T40N, T40XP, T41NQ, T41LQ and A1N; `ingenic-census-test.c` fails if the
  model at reset reads differently.
- Other tests pin CRC32s, ramps and register values of an engine (LDC,
  JPEG, IPU, I2D, HLDC, CPM, UART and others) to a part; each names its
  origin in its header. The "Silicon corpora" section of
  [../README.md](../README.md) lists them and what was left out.
- Address-map facts (bootroms, the unmapped windows, RAM aliases) are
  checked by `ingenic-bootrom-test.c`, `ingenic-boot-test.c` and
  `ingenic-busregs-test.c`.

The T33 has no census, so its pages show no block as register-accurate
from it.

## Keeping the pages current

`contrib/ingenic/devdoc.py` rewrites the generated pages from a built
`qemu-system-mipsel`; `contrib/ingenic/devdoc.py --check` writes nothing
and exits 1 if any page differs. The Linux x86_64 CI job runs it after
the build.
