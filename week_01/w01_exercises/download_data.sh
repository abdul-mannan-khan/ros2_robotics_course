#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${SCRIPT_DIR}/bag_data"
DL="${BAG_DIR}/kitti_raw"
mkdir -p "${DL}"
echo "=== Week 1: KITTI Raw LiDAR Download ==="
URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0001/2011_09_26_drive_0001_sync.zip"
ZIP="${DL}/kitti_sync.zip"
if [ ! -f "${ZIP}" ]; then
    echo "Downloading..."
    wget -q --show-progress -O "${ZIP}" "${URL}"
else
    echo "Already downloaded."
fi
if [ ! -d "${DL}/2011_09_26" ]; then
    echo "Extracting..."
    unzip -q "${ZIP}" -d "${DL}"
else
    echo "Already extracted."
fi
echo "Done. Run generate_bag.py for ROS2 bag."
