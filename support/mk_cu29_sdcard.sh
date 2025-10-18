#!/usr/bin/env bash
set -euo pipefail

# Cu29 GPT SD creator:
# - GPT
# - p1: 1 MiB FAT12, PARTLABEL=COPPERCFG
# - p2: rest, PARTLABEL=Cu29, TYPEGUID=29A2E0C9-0000-4C75-9229-000000000029

usage(){ echo "usage: $0 /dev/sdX_or_/dev/mmcblkY"; exit 1; }
[[ $# -eq 1 ]] || usage
DEV="$1"; [[ -b "$DEV" ]] || { echo "not a block device: $DEV"; exit 1; }

# tools check
for t in sgdisk mkfs.vfat lsblk wipefs udevadm; do
  command -v "$t" >/dev/null || { echo "missing tool: $t"; exit 1; }
done

read -r -p "ERASE ALL DATA on $DEV? type 'YES' to continue: " ans
[[ "$ans" == "YES" ]] || { echo "aborted"; exit 1; }

# best-effort unmount children
lsblk -ln "$DEV" -o NAME,MOUNTPOINTS | while read -r name mp; do
  [[ -n "${mp:-}" ]] && sudo umount -f "/dev/$name" || true
done

# wipe old signatures and tables
sudo wipefs -a "$DEV" || true
sudo sgdisk --zap-all "$DEV"

# create fresh GPT with:
#  - p1: 1MiB, 0700 Basic Data, PARTLABEL=COPPERCFG
#  - p2: rest, custom Cu29 TYPEGUID, PARTLABEL=Cu29
CU29_GUID="29A2E0C9-0000-4C75-9229-000000000029"
sudo sgdisk -og "$DEV"
sudo sgdisk \
  -n1:1MiB:+1MiB  -t1:0700                                -c1:"COPPERCFG" \
  -n2:0:0         -t2:${CU29_GUID}                        -c2:"Cu29" \
  "$DEV"

# settle device nodes
sudo partprobe "$DEV" || true
sudo udevadm settle || true
sleep 0.5

# derive part paths
if [[ "$DEV" =~ [0-9]$ ]]; then P1="${DEV}p1"; P2="${DEV}p2"; else P1="${DEV}1"; P2="${DEV}2"; fi
[[ -b "$P1" && -b "$P2" ]] || { echo "partition nodes not found: $P1 $P2"; exit 1; }

# format tiny p1 as FAT12 (FAT16 is too large for 1MiB)
sudo mkfs.vfat -F 12 -n COPPERCFG "$P1"

# mount and write README
mnt=$(mktemp -d)
trap 'sudo umount "$mnt" 2>/dev/null || true; rmdir "$mnt" 2>/dev/null || true' EXIT
sudo mount "$P1" "$mnt"
sudo tee "$mnt/README-COPPER.txt" >/dev/null <<'TXT'
This SD card uses Copper (Cu29) raw binary logging.

Partition layout (GPT):
  - Partition 1 (FAT12, label COPPERCFG): configuration + this notice
  - Partition 2 (type GUID 29A2E0C9-0000-4C75-9229-000000000029, label Cu29): Copper raw log storage

To read logs, use the Copper log reader.
Do NOT format or mount partition 2 — it is not a filesystem.
TXT
sync
sudo umount "$mnt"
rmdir "$mnt"
trap - EXIT

# show result
lsblk -o NAME,SIZE,TYPE,FSTYPE,PTTYPE,PARTLABEL,PARTTYPE,LABEL "$DEV"
echo "Done. Copper raw partition is $P2 (GPT type ${CU29_GUID}, PARTLABEL=Cu29)."
