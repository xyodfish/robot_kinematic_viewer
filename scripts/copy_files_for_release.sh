#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
SRC_DIR="$SCRIPT_DIR/../robot_kinematic_viewer_release"
TARGET_DIR="${RKV_RELEASE_TARGET_DIR:-$SCRIPT_DIR/../../robot_kinematic_viewer_release}"

if [ ! -d "$SRC_DIR" ]; then
    echo "❌ 错误: 源目录不存在: $SRC_DIR"
    echo "请先执行 scripts/collect_dep.sh（通常由 auto_build.sh 调用）。"
    exit 1
fi

mkdir -p "$TARGET_DIR"
rm -rf "$TARGET_DIR"/*

cp -a "$SRC_DIR"/. "$TARGET_DIR"/
find "$TARGET_DIR" -type f -name "*.sh" -exec chmod +x {} \;

echo "✅ 已复制发布目录到: $TARGET_DIR"
