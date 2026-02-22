#!/bin/bash
# CAN bus setup for RobStride motors
# can0 = Left arm (RS-01/02/03/04)
# can1 = Right arm (RS-01/02/03/04)
#
# RobStride motors typically use 1Mbit/s CAN bus.
# MODOSITSD a bitrate-et ha a motorjaid mast hasznalnak!

set -e

BITRATE=${1:-1000000}  # Default: 1 Mbit/s
CANFD=${2:-off}         # CAN FD: on/off

echo "=== CAN Bus Setup ==="
echo "Bitrate: ${BITRATE}"
echo "CAN FD: ${CANFD}"

for iface in can0 can1; do
    echo "--- Setting up ${iface} ---"

    # Bring down if already up
    sudo ip link set ${iface} down 2>/dev/null || true

    if [ "${CANFD}" = "on" ]; then
        # CAN FD mode (if RobStride supports it)
        sudo ip link set ${iface} type can \
            bitrate ${BITRATE} \
            dbitrate 5000000 \
            fd on \
            restart-ms 100
    else
        # Classic CAN mode
        sudo ip link set ${iface} type can \
            bitrate ${BITRATE} \
            restart-ms 100
    fi

    # Set TX queue length (higher = less drops under load)
    sudo ip link set ${iface} txqueuelen 1000

    # Bring up
    sudo ip link set ${iface} up

    # Verify
    STATE=$(cat /sys/class/net/${iface}/operstate 2>/dev/null || echo "unknown")
    echo "${iface}: state=${STATE}, bitrate=${BITRATE}"
done

echo ""
echo "=== CAN Bus Status ==="
ip -brief link show type can
echo ""
echo "=== Verify with: candump can0 (or can1) ==="
echo "=== Send test:   cansend can0 001#0102030405060708 ==="
