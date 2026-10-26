.. _MIPS-System-emulator:

MIPS System emulator
--------------------

Four executables cover simulation of 32 and 64-bit MIPS systems in both
endian options, ``qemu-system-mips``, ``qemu-system-mipsel``
``qemu-system-mips64`` and ``qemu-system-mips64el``. Five different
machine types are emulated:

-  The MIPS Malta prototype board \"malta\"

-  An ACER Pica \"pica61\". This machine needs the 64-bit emulator.

-  A MIPS Magnum R4000 machine \"magnum\". This machine needs the
   64-bit emulator.

The Malta emulation supports the following devices:

-  Core board with MIPS 24Kf CPU and Galileo system controller

-  PIIX4 PCI/USB/SMbus controller

-  The Multi-I/O chip's serial device

-  PCI network cards (PCnet32 and others)

-  Malta FPGA serial device

-  Cirrus (default) or any other PCI VGA graphics card

The Boston board emulation supports the following devices:

-  Xilinx FPGA, which includes a PCIe root port and an UART

-  Intel EG20T PCH connects the I/O peripherals, but only the SATA bus
   is emulated

The ACER Pica emulation supports:

-  MIPS R4000 CPU

-  PC-style IRQ and DMA controllers

-  PC Keyboard

-  IDE controller

The MIPS Magnum R4000 emulation supports:

-  MIPS R4000 CPU

-  PC-style IRQ controller

-  PC Keyboard

-  SCSI controller

-  G364 framebuffer

The Fuloong 2E emulation supports:

-  Loongson 2E CPU

-  Bonito64 system controller as North Bridge

-  VT82C686 chipset as South Bridge

-  RTL8139D as a network card chipset

The Loongson-3 virtual platform emulation supports:

-  Loongson 3A CPU

-  LIOINTC as interrupt controller

-  GPEX and virtio as peripheral devices

Ingenic XBurst SoCs
~~~~~~~~~~~~~~~~~~~

Eight XBurst1 single-core MIPS32r2 SoCs (T10, T20, T21, T23, T30, T31,
T32, T33) and three XBurst2 dual-core SMP SoCs (A1, T40, T41) used in
IP cameras, embedded recorders and IoT devices.

QEMU machine names match the part numbers: ``ingenic-t10`` through
``ingenic-t33``, plus ``ingenic-a1``, ``ingenic-t40`` and
``ingenic-t41``. Each XBurst1 machine has subvariants selectable via
``-global ingenic-t31.soc-variant=<name>`` (for example, ``t31x``,
``t32nq``, ``t20l``); subvariant controls cpuid, DDR geometry and the
clocking configuration. The C100 runs as a T31 variant (``c100``), the
T32Pro and CW080 as T32 variants (``t32prozn``, ``cw080sn`` and others).
``-m`` is the board's DDR: at most 256 MiB on an XBurst1 SoC, below its
peripherals; on the T40, T41 and A1 (up to 512 MiB, 512 MiB and 1 GiB)
all of it shows again from 0x20000000. QEMU warns when ``-m`` is less
than the subvariant has.

Supported peripherals:

-  CPM (clock and PLL control), DDR2/DDR3 controller with InnoSilicon
   PHY
-  SPI-NOR flash controller (V1 on T10-T31, V2 on T32/T33/XBurst2),
   its part 16, 32 or 64 MiB to fit the image (``jedec-id`` sets its ID)
-  Ethernet: vendor MAC (older dwmac) on T-series and T40/T41,
   dwxgmac2 on A1
-  DWC2 USB host (3 controllers on A1, one on T40/T41 and the
   T-series)
-  TCU + OST register page: 8 timer/PWM channels, watchdog (channel
   16) and the embedded 64-bit OST counter
-  Dedicated PWM controller (V2) at 0x13450000 on T32/T33 and at
   0x13460000 on A1/T40/T41
-  Real-time clock, true random number generator, I2C, SDHCI/MSC,
   PDMA, GPIO, SADC, INTC and UART
-  I2C0 bit-banged through its pads as GPIOs on the T-series, T40 and
   T41, as vendor factory probes and the T10's kernels do
-  SDIO cards on the MSC and SDHCI controllers, whose buses are named
   ``msc0`` and ``msc1`` on the T-series, T40 and T41
-  Wi-Fi as the cameras' Linux drivers see it: AICSemi AIC8800,
   AIC8800D80, AIC8800DC and AIC8800DW on USB and AIC8800D80 on SDIO,
   AltoBeam ATBM60xx on SDIO or USB, Wi-Fi 6 parts included, Broadcom
   BCM43438 and BCM43436 on SDIO, HiSilicon Hi3881 on SDIO and WS73V100
   on USB, Huge-IC TXW901U on USB, iComm-semi SSV6155P and SSV6355 on
   USB and SSV6158 on SDIO, MediaTek MT7601U on USB, Realtek RTL8189FS,
   RTL8189ES and RTL8192FS on SDIO and RTL8188FU, RTL8188EUS and
   RTL8733BU on USB, Seekwave SWT6621S on USB and WuQi WQ9001 on USB,
   with a WPA2 access point to join
   (``-device atbm6031,bus=msc1,netdev=...``) and a client for the
   chip's own access point (``client-netdev=...``)
-  SPI NAND on the flash controller, and the LZMA engines of the T32,
   T33, T40 and T41
-  AES, DES, HASH and RSA engines; the RSA engine needs GMP to build
-  Camera video on every T-series SoC and the T40 and T41: image
   sensors on I2C (``-device image-sensor``), every model thingino's
   cameras and the vendor SDK demos use, the ISP with its OSD, the IPU,
   the IVDC line ring and the H.264, H.265 and JPEG encoders
-  Audio: the AIC with the internal codec, and the DMIC of the T30,
   T31, T32, T33, T40 and T41
-  AHCI SATA (A1 only)
-  Per-core INTC, Core OST and CCU on the XBurst2 family

Blocks without a model, such as the NPUs, the LDC and the display
outputs, are unimplemented stubs. The Ethernet MAC takes the netdev
named ``n0``. ``contrib/ingenic/README.md``, shipped with the release
binaries, covers video, audio, SPI NOR and NAND, the test inputs
(``-global ingenic-gpio.in-pb=...``), the pin trace
(``-global ingenic-gpio.trace=<chardev>``) and Wi-Fi in detail.

Example - boot a flash image::

   qemu-system-mipsel -M ingenic-t31 -m 256M -display none \
       -serial null -serial stdio \
       -drive file=<flash_image>.bin,format=raw,if=none,id=flash0

Example - a T31 camera with its GC2053 sensor, its RTSP port forwarded
to the host::

   qemu-system-mipsel -M ingenic-t31 -m 128M -display none \
       -serial null -serial stdio \
       -drive file=<flash_image>.bin,format=raw,if=none,id=flash0 \
       -netdev user,id=n0,hostfwd=tcp::8554-:554 \
       -device image-sensor,model=gc2053,bus=i2c0

.. include:: cpu-models-mips.rst.inc

.. _nanoMIPS-System-emulator:

nanoMIPS System emulator
~~~~~~~~~~~~~~~~~~~~~~~~

Executable ``qemu-system-mipsel`` also covers simulation of 32-bit
nanoMIPS system in little endian mode:

-  nanoMIPS I7200 CPU

Example of ``qemu-system-mipsel`` usage for nanoMIPS is shown below:

Download ``<disk_image_file>`` from
https://mipsdistros.mips.com/LinuxDistro/nanomips/buildroot/index.html.

Download ``<kernel_image_file>`` from
https://mipsdistros.mips.com/LinuxDistro/nanomips/kernels/v4.15.18-432-gb2eb9a8b07a1-20180627102142/index.html.

Start system emulation of Malta board with nanoMIPS I7200 CPU::

   qemu-system-mipsel -cpu I7200 -kernel <kernel_image_file> \
       -M malta -serial stdio -m <memory_size> -drive file=<disk_image_file>,format=raw \
       -append "mem=256m@0x0 rw console=ttyS0 vga=cirrus vesa=0x111 root=/dev/sda"
