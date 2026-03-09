#!/bin/bash
# setup_vcan.sh — Create a virtual CAN interface for testing
#
# Usage:
#   sudo ./setup_vcan.sh          # Create vcan0
#   sudo ./setup_vcan.sh vcan1    # Create a custom-named interface
#   sudo ./setup_vcan.sh --down   # Tear down vcan0

set -euo pipefail

IFACE="${1:-vcan0}"

if [ "$IFACE" = "--down" ]; then
    IFACE="vcan0"
    echo "Tearing down $IFACE..."
    ip link set down "$IFACE" 2>/dev/null || true
    ip link delete "$IFACE" type vcan 2>/dev/null || true
    echo "Done."
    exit 0
fi

echo "Setting up virtual CAN interface: $IFACE"

# Load the vcan kernel module
if ! lsmod | grep -q '^vcan'; then
    echo "Loading vcan kernel module..."
    modprobe vcan
fi

# Create the interface if it doesn't exist
if ! ip link show "$IFACE" &>/dev/null; then
    echo "Creating $IFACE..."
    ip link add dev "$IFACE" type vcan
fi

# Bring it up
ip link set up "$IFACE"

echo "Virtual CAN interface $IFACE is ready."
echo ""
echo "Test with:"
echo "  candump $IFACE &"
echo "  cansend $IFACE 123#DEADBEEF"
echo ""
echo "Install can-utils if needed:"
echo "  apt install can-utils"
