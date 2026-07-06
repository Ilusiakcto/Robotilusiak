#!/bin/bash
# Bring up CANable2 / gs_usb CAN FD interfaces for OpenArm teleop.
#
# Works on:
#   - x86_64 / modern kernels: gs_usb is already in-tree, so this just loads
#     the module and brings the interfaces up (no build).
#   - Jetson (stock kernel lacks gs_usb FD): builds the module from source
#     the first time, then loads it.
#
# Usage:
#   sudo ./reinit_can.sh                 # default interfaces: can0 can1
#   sudo ./reinit_can.sh can2 can3       # Jetson: native mttcan owns can0/can1

set -e

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo $0 $*"
    exit 1
fi

# Interfaces to bring up (default can0 can1)
IFACES=("$@")
if [ ${#IFACES[@]} -eq 0 ]; then
    IFACES=(can0 can1)
fi

BITRATE=1000000
DBITRATE=5000000

# 1. Ensure gs_usb is loaded. Prefer the in-tree module; only build from
#    source if this kernel doesn't ship it (the Jetson case).
if ! lsmod | grep -q '^gs_usb'; then
    if modprobe gs_usb 2>/dev/null; then
        echo "Loaded in-tree gs_usb."
    else
        echo "gs_usb not available in this kernel — building from source..."
        KVER=$(uname -r)
        BUILD_DIR="/tmp/gs_usb_build"
        MODULE_DIR="/lib/modules/$KVER/kernel/drivers/net/can/usb"
        mkdir -p "$BUILD_DIR"
        curl -sL "https://raw.githubusercontent.com/torvalds/linux/v5.19/drivers/net/can/usb/gs_usb.c" \
            -o "$BUILD_DIR/gs_usb.c"
        echo 'obj-m += gs_usb.o' > "$BUILD_DIR/Makefile"
        # Out-of-tree build: pass the module dir explicitly via M= (absolute).
        make -C "/lib/modules/$KVER/build" M="$BUILD_DIR" modules
        mkdir -p "$MODULE_DIR"
        cp "$BUILD_DIR/gs_usb.ko" "$MODULE_DIR/gs_usb.ko"
        depmod -a
        echo "gs_usb" > /etc/modules-load.d/gs_usb.conf
        modprobe gs_usb
        echo "Built and loaded gs_usb."
    fi
fi

# 2. Bring up each requested interface as CAN FD.
for IFACE in "${IFACES[@]}"; do
    # Give the interface a moment to appear after the module loads.
    for _ in $(seq 1 10); do
        ip link show "$IFACE" &>/dev/null && break
        sleep 0.3
    done
    if ! ip link show "$IFACE" &>/dev/null; then
        echo "ERROR: $IFACE not found. Is the CANable2 adapter plugged in?"
        echo "       Detected CAN interfaces: $(ls /sys/class/net 2>/dev/null | grep -E '^can' | tr '\n' ' ')"
        exit 1
    fi
    ip link set "$IFACE" down 2>/dev/null || true
    ip link set "$IFACE" up type can bitrate "$BITRATE" dbitrate "$DBITRATE" fd on
    echo "$IFACE up (CAN FD ${BITRATE}/${DBITRATE})"
done

echo "CAN interfaces ready:"
for IFACE in "${IFACES[@]}"; do
    ip -details link show "$IFACE" | head -2
done
