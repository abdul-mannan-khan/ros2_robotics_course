#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd $(dirname ${BASH_SOURCE[0]}) && pwd)
DATA_DIR=${SCRIPT_DIR}/bag_data

echo "===== Week 12: nuScenes Mini Dataset Downloader ====="
mkdir -p ${DATA_DIR}/nuscenes_mini
cd ${DATA_DIR}/nuscenes_mini

if [ ! -f nuscenes.done ]; then
    echo "[INFO] Downloading nuScenes mini..."
    echo "[NOTE] nuScenes requires registration at https://www.nuscenes.org/"
    echo "[NOTE] Generating synthetic data instead..."
    cd ${SCRIPT_DIR} && python3 generate_bag.py
    exit 0
fi

echo "[DONE]"
