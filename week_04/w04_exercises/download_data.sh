#!/usr/bin/env bash
set -euo pipefail
SD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BD="${SD}/bag_data"
mkdir -p "${BD}"
URL="https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag"
R1="${BD}/deutsches_museum_2d.bag"
R2="${BD}/deutsches_museum_2d"
echo "=== Week 4: Cartographer Deutsches Museum 2D ==="
if [ -d "${R2}" ]; then echo "Already exists."; exit 0; fi
echo "Downloading (~500MB)..."
wget -c -O "${R1}" "${URL}" || curl -L -o "${R1}" "${URL}" || { echo "Failed"; exit 1; }
echo "Converting ROS1->ROS2..."
if python3 -c "import rosbags" 2>/dev/null; then
  rosbags-convert "${R1}" --dst "${R2}" && rm -f "${R1}"
else
  echo "pip3 install rosbags OR use generate_bag.py"
fi
echo "Done."
