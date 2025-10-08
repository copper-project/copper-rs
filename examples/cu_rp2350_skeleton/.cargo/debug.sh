#!/usr/bin/env bash
set -euo pipefail
ELF="$1"
picotool load -u -v -x -t elf "$ELF"
DEV="${DEV:-$(ls -1 /dev/serial/by-id/usb-1a86_* | head -n1)}"
exec socat -u "$DEV" - | defmt-print -e "$ELF"
