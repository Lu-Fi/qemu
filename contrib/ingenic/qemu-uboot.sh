#!/bin/bash
# qemu-uboot.sh - Start/stop QEMU T31 and interact with U-Boot
#
# Usage:
#   qemu-uboot.sh start [flash.bin]  - Start QEMU, wait for U-Boot prompt
#   qemu-uboot.sh stop               - Kill QEMU
#   qemu-uboot.sh cmd "command"      - Send command, return output
#   qemu-uboot.sh status             - Check if running
#
# Serial is on a Unix socket at /tmp/qemu-t31.sock
# All output logged to /tmp/qemu-t31-serial.log

QEMU_BIN="$HOME/projects/thingino/qemu-project/qemu/build/qemu-system-mipsel"
KERNEL="$HOME/projects/thingino/ingenic-u-boot-xburst1/spl/u-boot-spl"
DEFAULT_FLASH="$HOME/projects/thingino/ingenic-u-boot-xburst1/u-boot-lzo-with-spl.bin"
SOCK="/tmp/qemu-t31.sock"
PIDFILE="/tmp/qemu-t31.pid"
SERIAL_LOG="/tmp/qemu-t31-serial.log"
READY_FILE="/tmp/qemu-t31-ready"

start_qemu() {
    local flash="${1:-}"
    local tmpflash="/tmp/qemu-t31-flash.bin"

    # Kill any existing instance
    stop_qemu 2>/dev/null

    # Prepare flash
    if [ -n "$flash" ] && [ -f "$flash" ]; then
        cp "$flash" "$tmpflash"
    elif [ -f "$DEFAULT_FLASH" ]; then
        cp "$DEFAULT_FLASH" "$tmpflash"
    else
        echo "ERROR: No flash image found"
        exit 1
    fi

    rm -f "$SOCK" "$READY_FILE" "$SERIAL_LOG"

    local sd_args=""
    if [ -n "$SD_IMAGE" ] && [ -f "$SD_IMAGE" ]; then
        sd_args="-drive if=sd,file=$SD_IMAGE,format=raw"
    fi

    # Start QEMU with serial on unix socket
    $QEMU_BIN \
        -M ingenic-t31 -m 128M \
        -kernel "$KERNEL" \
        -drive file="$tmpflash",format=raw,if=none \
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

    local script_dir="$(cd "$(dirname "$0")" && pwd)"
    expect "$script_dir/qemu-boot.exp" 180 2>/dev/null

    local boot_result=$(cat /tmp/qemu-t31-boot-result.txt 2>/dev/null)
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
    local script_dir="$(cd "$(dirname "$0")" && pwd)"

    if [ ! -S "$SOCK" ]; then
        echo "ERROR: QEMU not running (no socket)"
        return 1
    fi

    expect "$script_dir/qemu-cmd.exp" "$cmd" "$timeout" 2>/dev/null
    cat /tmp/qemu-t31-cmd-result.txt
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
