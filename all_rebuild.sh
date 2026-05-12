#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
BUILD_DIR="${RKV_BUILD_DIR:-$SCRIPT_DIR/build}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Debug}"

echo "========== 全量重编译 robot_kinematic_viewer =========="
rm -rf "$BUILD_DIR"
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$BUILD_DIR" --target robot_kinematic_viewer -j"$(nproc)"
echo "✅ 全量重编译完成"
