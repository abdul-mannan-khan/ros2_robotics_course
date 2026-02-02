#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd $(dirname ${BASH_SOURCE[0]}) && pwd)
DATA_DIR=${SCRIPT_DIR}/bag_data
KITTI_URL=https://s3.eu-central-1.amazonaws.com/avg-kitti

echo "===== Week 10: KITTI Tracking Dataset Downloader ====="
mkdir -p ${DATA_DIR}/kitti_tracking && cd ${DATA_DIR}/kitti_tracking

for name in data_tracking_image_2 data_tracking_image_3 data_tracking_label_2 data_tracking_velodyne; do
    if [ ! -f ${name}.done ]; then
        echo "[INFO] Downloading ${name}..."
        wget -q --show-progress ${KITTI_URL}/${name}.zip -O ${name}.zip && \
            unzip -qo ${name}.zip && rm ${name}.zip && touch ${name}.done || \
            echo "[WARN] ${name} download failed."
    fi
done

echo "[INFO] Converting to ROS2 bag..."
cd ${SCRIPT_DIR} && python3 generate_bag.py --kitti-dir ${DATA_DIR}/kitti_tracking --sequence 0000
echo "[DONE] Bag written."
