#!/bin/bash
# qemu-uboot.sh - Start/stop QEMU T31 and interact with U-Boot
#
# Usage:
#   qemu-uboot.sh start [flash.bin]  - Start QEMU, wait for U-Boot prompt
#   qemu-uboot.sh stop               - Kill QEMU
#   qemu-uboot.sh cmd "command"      - Send command, return output
#   qemu-uboot.sh status             - Check if running
#
# Env overrides:
#   QEMU_BIN       path to qemu-system-mipsel (auto-detected if unset)
#   KERNEL         path to u-boot-spl ELF (REQUIRED)
#   DEFAULT_FLASH  default flash image (used when no arg passed to start)
#   SD_IMAGE       optional SD-card backing file
#
# Serial is on a Unix socket at $SOCK (default /tmp/qemu-t31.sock).
# All output logged to $SERIAL_LOG (default /tmp/qemu-t31-serial.log).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Auto-locate qemu-system-mipsel like qemu-bsp.sh does.
if [ -z "${QEMU_BIN:-}" ]; then
    if   [ -x "$SCRIPT_DIR/qemu-system-mipsel" ];        then QEMU_BIN="$SCRIPT_DIR/qemu-system-mipsel"
    elif [ -x "$SCRIPT_DIR/qemu-system-mipsel.exe" ];    then QEMU_BIN="$SCRIPT_DIR/qemu-system-mipsel.exe"
    elif [ -x "$SCRIPT_DIR/../../build/qemu-system-mipsel" ]; then QEMU_BIN="$SCRIPT_DIR/../../build/qemu-system-mipsel"
    else QEMU_BIN="$(command -v qemu-system-mipsel 2>/dev/null || true)"
    fi
fi

KERNEL="${KERNEL:-}"
DEFAULT_FLASH="${DEFAULT_FLASH:-}"
SOCK="${SOCK:-/tmp/qemu-t31.sock}"
PIDFILE="${PIDFILE:-/tmp/qemu-t31.pid}"
SERIAL_LOG="${SERIAL_LOG:-/tmp/qemu-t31-serial.log}"
READY_FILE="${READY_FILE:-/tmp/qemu-t31-ready}"
BOOT_RESULT="${BOOT_RESULT:-/tmp/qemu-t31-boot-result.txt}"
CMD_RESULT="${CMD_RESULT:-/tmp/qemu-t31-cmd-result.txt}"
TMP_FLASH="${TMP_FLASH:-/tmp/qemu-t31-flash.bin}"

[ -n "${QEMU_BIN:-}" ] && [ -e "$QEMU_BIN" ] || {
    echo "ERROR: qemu-system-mipsel not found; set QEMU_BIN=..." >&2
    exit 1
}
[ -n "$KERNEL" ] && [ -e "$KERNEL" ] || {
    echo "ERROR: u-boot-spl not found; set KERNEL=/path/to/u-boot-spl" >&2
    exit 1
}

start_qemu() {
    local flash="${1:-}"

    # Kill any existing instance
    stop_qemu 2>/dev/null

    # Prepare flash
    if [ -n "$flash" ] && [ -f "$flash" ]; then
        cp "$flash" "$TMP_FLASH"
    elif [ -n "$DEFAULT_FLASH" ] && [ -f "$DEFAULT_FLASH" ]; then
        cp "$DEFAULT_FLASH" "$TMP_FLASH"
    else
        echo "ERROR: No flash image; pass one as arg or set DEFAULT_FLASH=..." >&2
        exit 1
    fi

    rm -f "$SOCK" "$READY_FILE" "$SERIAL_LOG"

    local sd_args=""
    if [ -n "$SD_IMAGE" ] && [ -f "$SD_IMAGE" ]; then
        sd_args="-drive if=sd,file=$SD_IMAGE,format=raw"
    fi

    # Start QEMU with serial on unix socket
    "$QEMU_BIN" \
        -M ingenic-t31 -m 128M \
        -kernel "$KERNEL" \
        -drive file="$TMP_FLASH",format=raw,if=none \
        $sd_args \
        -serial null \
        -serial unix:"$SOCK",server=on,wait=off \
        -display none \
        -netdev user,id=n0 \
        -daemonize \
        -pidfile "$PIDFILE" 2>/dev/null

    if [ $? -ne 0 ]; then
        echo "ERROR: QEMU failed to start"
        exit 1
    fi

    echo "QEMU started, PID=$(cat $PIDFILE 2>/dev/null)"
    echo "Waiting for U-Boot prompt..."

    SOCK="$SOCK" RESULT_FILE="$BOOT_RESULT" SERIAL_LOG="$SERIAL_LOG" \
        expect "$SCRIPT_DIR/qemu-boot.exp" 180 2>/dev/null

    local boot_result=$(cat "$BOOT_RESULT" 2>/dev/null)
    if echo "$boot_result" | grep -q "READY"; then
        touch "$READY_FILE"
        echo "U-Boot prompt ready"
        return 0
    else
        echo "FAILED: $boot_result"
        tail -5 "$SERIAL_LOG" 2>/dev/null
        return 1
    fi
}

stop_qemu() {
    if [ -f "$PIDFILE" ]; then
        kill -9 "$(cat $PIDFILE)" 2>/dev/null
        rm -f "$PIDFILE"
    fi
    kill -9 $(pgrep -f "qemu-system-mipsel.*ingenic-t31") 2>/dev/null
    rm -f "$SOCK" "$READY_FILE"
    echo "QEMU stopped"
}

send_cmd() {
    local cmd="$1"
    local timeout="${2:-10}"

    if [ ! -S "$SOCK" ]; then
        echo "ERROR: QEMU not running (no socket)"
        return 1
    fi

    SOCK="$SOCK" RESULT_FILE="$CMD_RESULT" \
        expect "$SCRIPT_DIR/qemu-cmd.exp" "$cmd" "$timeout" 2>/dev/null
    cat "$CMD_RESULT"
}

check_status() {
    if [ -f "$PIDFILE" ] && kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
        echo "RUNNING (PID=$(cat $PIDFILE))"
        if [ -f "$READY_FILE" ]; then
            echo "U-Boot prompt: READY"
        else
            echo "U-Boot prompt: NOT READY"
        fi
        echo "Serial log: $(wc -l < $SERIAL_LOG 2>/dev/null || echo 0) lines"
        return 0
    else
        echo "NOT RUNNING"
        return 1
    fi
}

case "${1:-}" in
    start)
        start_qemu "$2"
        ;;
    stop)
        stop_qemu
        ;;
    cmd)
        send_cmd "$2" "${3:-10}"
        ;;
    status)
        check_status
        ;;
    log)
        tail -${2:-20} "$SERIAL_LOG" 2>/dev/null
        ;;
    *)
        echo "Usage: $0 {start [flash]|stop|cmd \"command\" [timeout]|status|log [lines]}"
        exit 1
        ;;
esac
