#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${SCRIPT_DIR}/bag_data"
DL="${BAG_DIR}/euroc"
mkdir -p "${DL}"
echo "=== Week 2: EuRoC MAV Dataset Download ==="
URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip"
ZIP="${DL}/V1_01_easy.zip"
if [ ! -f "${ZIP}" ]; then
    echo "Downloading EuRoC V1_01_easy..."
    wget -q --show-progress -O "${ZIP}" "${URL}"
else
    echo "Already downloaded."
fi
if [ ! -d "${DL}/mav0" ]; then
    echo "Extracting..."
    unzip -q "${ZIP}" -d "${DL}"
else
    echo "Already extracted."
fi
echo "Done. Run generate_bag.py for ROS2 bag."
