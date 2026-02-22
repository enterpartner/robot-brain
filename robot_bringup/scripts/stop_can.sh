#!/bin/bash
# Shutdown CAN interfaces safely
set -e

echo "=== Shutting down CAN interfaces ==="
for iface in can0 can1; do
    sudo ip link set ${iface} down 2>/dev/null && \
        echo "${iface}: DOWN" || \
        echo "${iface}: already down"
done
