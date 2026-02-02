#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${SCRIPT_DIR}/bag_data"
echo "=== Week 7: PX4 SITL Bag Generation ==="
echo "No external dataset required."
python3 "${SCRIPT_DIR}/generate_bag.py" --output-dir "${BAG_DIR}"
echo "Done."
