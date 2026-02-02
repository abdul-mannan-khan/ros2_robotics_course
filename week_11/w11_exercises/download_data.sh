#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd $(dirname ${BASH_SOURCE[0]}) && pwd)
DATA_DIR=${SCRIPT_DIR}/bag_data
EUROC_URL=http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy
echo "===== Week 11: EuRoC MAV Dataset Downloader ====="
mkdir -p ${DATA_DIR}/euroc_mh01 && cd ${DATA_DIR}/euroc_mh01
if [ ! -f euroc.done ]; then
    echo "[INFO] Downloading EuRoC MH_01_easy..."
    wget -q --show-progress ${EUROC_URL}/MH_01_easy.zip -O MH_01_easy.zip && unzip -qo MH_01_easy.zip && rm MH_01_easy.zip && touch euroc.done || { echo "[WARN] Download failed. Generating synthetic."; cd ${SCRIPT_DIR} && python3 generate_bag.py; exit 0; }
fi
echo "[INFO] Converting to ROS2 bag..."
cd ${SCRIPT_DIR} && python3 generate_bag.py --euroc-dir ${DATA_DIR}/euroc_mh01
echo "[DONE]"
