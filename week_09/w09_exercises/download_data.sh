#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${SCRIPT_DIR}/bag_data"
mkdir -p "${BAG_DIR}"
echo "=== Week 9: EGO-Planner Sim Data ==="
echo "Generating synthetic depth and odometry data..."
python3 "${SCRIPT_DIR}/generate_bag.py" --output-dir "${BAG_DIR}"
echo "Done."
