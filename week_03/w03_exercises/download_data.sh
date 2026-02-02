#!/usr/bin/env bash
set -euo pipefail
echo "=== Week 3: No download needed ==="
echo "Run generate_bag.py to create synthetic TurtleBot3 data."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "${SCRIPT_DIR}/generate_bag.py"
