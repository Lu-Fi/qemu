#!/bin/bash
# Run the Ingenic T31 QEMU machine.
#
# USAGE
#   qemu-bsp.sh [OPTIONS]
#
# BOOT MODE (mutually exclusive)
#   -f, --flash FILE          Boot from a full flash image only (no -kernel)
#   -i, --image FILE          Alias for --flash; nicer to remember
#   -u, --uboot               Boot through SPL+U-Boot (loads $UBOOT)
#   (default)                 Boot Linux directly from $KERNEL using $FLASH for storage
#
# DEVICE ATTACHMENTS
#   -S, --usb-storage [FILE]  Attach usb-storage at port=1 (avoids QEMU's auto-hub).
#                             FILE defaults to /tmp/usb-stick.img.
#
# CONSOLE / IO
#   -c, --console-socket PATH Expose UART1 (/dev/ttyS1, the Linux console) as a
#                             UNIX socket instead of stdio. Use with the qemu_drive.py
#                             helper or socat.
#   -T, --console-tcp PORT    Expose UART1 as TCP localhost:PORT. Pairs with the
#                             /uart skill (device "qemu_t31", port 3331 by default).
#       --console-stdio       Force stdio (default unless -c/-T given).
#
# NETWORK
#   -p, --ssh-port PORT       SSH forward host:PORT -> guest:22 (default 2230).
#       --no-ssh              Don't add an SSH hostfwd.
#
# SOC / MACHINE
#   -M, --machine NAME        QEMU machine type (default: ingenic-t31).
#                             Auto-detected from -V if not specified:
#                             t10* -> ingenic-t10, t20* -> ingenic-t20,
#                             t21* -> ingenic-t21, else ingenic-t31.
#   -V, --variant NAME        Select T31 sub-model for EFUSE identification.
#                             libimp and thingino's soc tool read this.
#                             Valid: t31n t31x t31l t31a t31al t31zl t31zx
#                                    t31zc t31lc qemu (default: qemu)
#
# OTHER
#   -m, --memory SIZE         Guest RAM (default 128M).
#   -E, --extra "ARGS"        Append raw args to the QEMU command line.
#   -n, --dry-run             Print the QEMU command and exit.
#   -h, --help                Show this help.
#
# ENV OVERRIDES
#   QEMU=...                  QEMU binary (default: build tree under this script)
#   KERNEL=...                Path to uImage (linux mode, default /tmp/uImage-uncomp)
#   UBOOT=...                 Path to u-boot-spl ELF (uboot mode)
#   FLASH=...                 Path to flash image (linux/uboot mode)
#   CMDLINE=...               Kernel command line (linux mode only)
#
# CANONICAL FLOWS
#   # Boot a thingino firmware, SSH on :2230, usb-storage attached:
#   qemu-bsp.sh -i /tmp/thingino-X.bin -S
#   ssh -p 2230 root@localhost              # password: root
#
#   # Same, but UART1 wired to TCP for the /uart skill:
#   qemu-bsp.sh -i /tmp/thingino-X.bin -T 3331
#   /uart start qemu_t31; /uart send qemu_t31 "uname -a"
#
#   # Scripted (no stdio), drive via UNIX socket:
#   qemu-bsp.sh -i /tmp/thingino-X.bin -c /tmp/qemu.sock
#   python3 qemu_drive.py < my-commands.txt
#
# Press C-a x to exit when on stdio.

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() { sed -n '2,/^# Press C-a x/p' "$0" | sed 's/^# \?//'; }

# defaults
MODE=linux
FLASH_ARG=""
USB_STORAGE_ARG=""
USB_STORAGE_DEFAULT=/tmp/usb-stick.img
SOC_VARIANT=""
MACHINE=""
CONSOLE_MODE=stdio
CONSOLE_PATH=""
CONSOLE_PORT=""
SSH_PORT=2230
NO_SSH=0
MEM="${MEM:-128M}"
EXTRA="${EXTRA:-}"
DRY_RUN=0

# Parse long options manually since getopts only does short.
while [ $# -gt 0 ]; do
    case "$1" in
        -u|--uboot)         MODE=uboot; shift ;;
        -f|--flash|-i|--image)
                            MODE=flash; FLASH_ARG="${2:-}"
                            [ -n "$FLASH_ARG" ] || { echo "$1 requires a path" >&2; exit 1; }
                            shift 2 ;;
        -S|--usb-storage)
                            # Optional argument: take next arg if it looks like a path.
                            if [ $# -ge 2 ] && [ -n "${2:-}" ] && [ "${2#-}" = "$2" ]; then
                                USB_STORAGE_ARG="$2"; shift 2
                            else
                                USB_STORAGE_ARG="$USB_STORAGE_DEFAULT"; shift
                            fi ;;
        -c|--console-socket)
                            CONSOLE_MODE=unix; CONSOLE_PATH="${2:-}"
                            [ -n "$CONSOLE_PATH" ] || { echo "$1 requires a path" >&2; exit 1; }
                            shift 2 ;;
        -T|--console-tcp)
                            CONSOLE_MODE=tcp; CONSOLE_PORT="${2:-}"
                            [ -n "$CONSOLE_PORT" ] || { echo "$1 requires a port" >&2; exit 1; }
                            shift 2 ;;
        --console-stdio)    CONSOLE_MODE=stdio; shift ;;
        -V|--variant)       SOC_VARIANT="${2:-}"
                            [ -n "$SOC_VARIANT" ] || { echo "$1 requires a name" >&2; exit 1; }
                            shift 2 ;;
        -M|--machine)       MACHINE="${2:-}"
                            [ -n "$MACHINE" ] || { echo "$1 requires a machine name" >&2; exit 1; }
                            shift 2 ;;
        -p|--ssh-port)      SSH_PORT="${2:-}"; shift 2 ;;
        --no-ssh)           NO_SSH=1; shift ;;
        -m|--memory)        MEM="${2:-}"; shift 2 ;;
        -E|--extra)         EXTRA="$EXTRA ${2:-}"; shift 2 ;;
        -n|--dry-run)       DRY_RUN=1; shift ;;
        -h|--help)          usage; exit 0 ;;
        --)                 shift; break ;;
        -*)                 echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
        *)                  echo "Unexpected argument: $1" >&2; exit 1 ;;
    esac
done

# Locate qemu-system-mipsel:
#   1. $QEMU env var
#   2. Same dir as this script (release tarball layout)
#   3. ../build/qemu-system-mipsel or ../qemu/build/...  (in-tree dev layout)
#   4. PATH lookup
if [ -z "${QEMU:-}" ]; then
    if   [ -x "$SCRIPT_DIR/qemu-system-mipsel" ];        then QEMU="$SCRIPT_DIR/qemu-system-mipsel"
    elif [ -x "$SCRIPT_DIR/qemu-system-mipsel.exe" ];    then QEMU="$SCRIPT_DIR/qemu-system-mipsel.exe"
    elif [ -x "$SCRIPT_DIR/../../build/qemu-system-mipsel" ]; then QEMU="$SCRIPT_DIR/../../build/qemu-system-mipsel"
    elif [ -x "$SCRIPT_DIR/qemu/build/qemu-system-mipsel" ];  then QEMU="$SCRIPT_DIR/qemu/build/qemu-system-mipsel"
    else QEMU="$(command -v qemu-system-mipsel 2>/dev/null || true)"
    fi
fi
[ -n "${QEMU:-}" ] && [ -e "$QEMU" ] || { echo "qemu-system-mipsel not found; set QEMU=/path/to/qemu-system-mipsel" >&2; exit 1; }

# Build the console (UART1) chardev.
case "$CONSOLE_MODE" in
    stdio) CONSOLE=( -serial null -serial stdio ) ;;
    unix)
        rm -f "$CONSOLE_PATH"
        CONSOLE=( -serial null
                  -serial "unix:$CONSOLE_PATH,server=on,wait=off" ) ;;
    tcp)
        CONSOLE=( -serial null
                  -serial "tcp:127.0.0.1:$CONSOLE_PORT,server=on,wait=off" ) ;;
esac

# SSH port forward.
if [ "$NO_SSH" = 1 ]; then
    NETDEV=( -netdev user,id=n0 )
else
    NETDEV=( -netdev "user,id=n0,hostfwd=tcp::$SSH_PORT-:22" )
fi

# usb-storage.
# Auto-detect machine type from variant if not explicitly set.
if [ -z "$MACHINE" ] && [ -n "$SOC_VARIANT" ]; then
    case "$SOC_VARIANT" in
        t10*) MACHINE=ingenic-t10 ;;
        t20*) MACHINE=ingenic-t20 ;;
        t21*) MACHINE=ingenic-t21 ;;
        t23*) MACHINE=ingenic-t23 ;;
        t30*) MACHINE=ingenic-t30 ;;
        t32*) MACHINE=ingenic-t32 ;;
        t33*) MACHINE=ingenic-t33 ;;
        *)    MACHINE=ingenic-t31 ;;
    esac
fi
MACHINE="${MACHINE:-ingenic-t31}"

# SoC variant.
VARIANT=()
if [ -n "$SOC_VARIANT" ]; then
    VARIANT=( -global "ingenic-t31.soc-variant=$SOC_VARIANT" )
fi

USB=()
if [ -n "$USB_STORAGE_ARG" ]; then
    [ -e "$USB_STORAGE_ARG" ] || {
        echo "missing usb-storage backing file: $USB_STORAGE_ARG" >&2; exit 1; }
    USB=( -device "usb-storage,port=1,drive=u0"
          -drive  "file=$USB_STORAGE_ARG,if=none,id=u0,format=raw" )
fi

# Boot mode.
case "$MODE" in
    flash)
        FLASH="${FLASH_ARG:-${FLASH:-}}"
        [ -n "$FLASH" ] || { echo "flash mode requires --flash/--image PATH" >&2; exit 1; }
        [ -e "$FLASH" ] || { echo "missing: $FLASH" >&2; exit 1; }
        BOOT=( -M "$MACHINE" -m "$MEM"
               -drive "file=$FLASH,format=raw,if=none" ) ;;
    uboot)
        FLASH="${FLASH:-/tmp/flash16-modded.bin}"
        UBOOT="${UBOOT:-$HOME/projects/thingino/ingenic-u-boot-xburst1/spl/u-boot-spl}"
        [ -e "$FLASH" ] || { echo "missing: $FLASH" >&2; exit 1; }
        [ -e "$UBOOT" ] || { echo "missing: $UBOOT" >&2; exit 1; }
        BOOT=( -M "$MACHINE" -m "$MEM"
               -kernel "$UBOOT"
               -drive "file=$FLASH,format=raw,if=none" ) ;;
    linux)
        FLASH="${FLASH:-/tmp/flash16-modded.bin}"
        KERNEL="${KERNEL:-/tmp/uImage-uncomp}"
        CMDLINE="${CMDLINE:-console=ttyS1,115200n8 mem=99M@0x0 log_buf_len=2M mtdparts=jz_sfc:256k(boot),32k(env),224k(config),1504k(kernel),6176k(rootfs),8192k@0x800000(extras) root=/dev/mtdblock4 rootfstype=squashfs ro}"
        [ -e "$FLASH" ]  || { echo "missing: $FLASH" >&2; exit 1; }
        [ -e "$KERNEL" ] || { echo "missing: $KERNEL" >&2; exit 1; }
        BOOT=( -M "$MACHINE" -m "$MEM"
               -kernel "$KERNEL" -append "$CMDLINE"
               -drive "file=$FLASH,format=raw,if=none" ) ;;
esac

CMD=( "$QEMU"
      "${BOOT[@]}"
      "${CONSOLE[@]}"
      -display none
      "${NETDEV[@]}"
      -monitor none
      "${USB[@]}"
      "${VARIANT[@]}" )

if [ -n "$EXTRA" ]; then
    # Split EXTRA on whitespace (intentional; matches the legacy shell-friendly behaviour).
    # shellcheck disable=SC2206
    EXTRA_ARR=( $EXTRA )
    CMD+=( "${EXTRA_ARR[@]}" )
fi

if [ "$DRY_RUN" = 1 ]; then
    printf '%q ' "${CMD[@]}"; echo
    exit 0
fi

exec "${CMD[@]}"
