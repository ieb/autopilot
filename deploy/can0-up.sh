#!/bin/bash
# Bring up a CAN interface at NMEA2000 bitrate (250 kbps).
# Called by udev (80-can.rules) when a USB-CAN adapter is plugged in,
# and can be run manually: sudo can0-up.sh can0

IFACE="${1:-can0}"
BITRATE=250000

/sbin/ip link set "$IFACE" type can bitrate "$BITRATE"
/sbin/ip link set "$IFACE" up

logger -t can0-up "Brought up $IFACE at ${BITRATE} bps"
