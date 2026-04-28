#!/bin/bash
# Profile with perf and generate report

set -e

BUILD_DIR="build-perf"
APP="$BUILD_DIR/src/Vortex"

cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$BUILD_DIR"

if [ ! -f "$APP" ]; then
  echo "Error: $APP not found. Build the project first."
  exit 1
fi

echo "=== Profiling with perf ==="
echo "Running application with perf record..."

# Record performance data
sudo perf record -g --call-graph dwarf -F 999 "$APP" --no-truth-csv "$@"

echo ""
echo "=== Performance Report ==="
sudo perf report --stdio | head -n 50

echo ""
echo "Full interactive report: sudo perf report"
echo "Flame graph: sudo perf script | stackcollapse-perf.pl | flamegraph.pl > flamegraph.svg"
