#!/usr/bin/env bash
# Week 8: 3D Path Planning
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${SCRIPT_DIR}/bag_data"
mkdir -p "${BAG_DIR}"
echo "=== Week 8: Newer College Dataset ==="
echo "Generating synthetic 3D pointcloud data..."
python3 "${SCRIPT_DIR}/generate_bag.py" --output-dir "${BAG_DIR}"
echo "Done."
