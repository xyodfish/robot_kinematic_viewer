#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

show_help() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -q, --quick   快速编译（增量 build.sh，不删 build）"
    echo "  --all         全量重编译（默认，等价 all_rebuild.sh）"
    echo "  -old          调用 switch_env.sh 时打印旧版环境提示"
    echo "  --skip-env    跳过 switch_env.sh"
    echo "  --self-test   打包后对发布目录执行 run.sh --self-test"
    echo "  -h, --help    显示帮助"
    echo ""
    echo "环境变量:"
    echo "  RKV_SKIP_SWITCH_ENV=1     跳过 switch_env"
    echo "  RKV_RELEASE_TARGET_DIR    发布拷贝目标目录（默认: OmniLink/robot_kinematic_viewer_release）"
    echo "  RKV_KEEP_STAGE_DIR=1      保留仓库内 staging: robot_kinematic_viewer_release"
    echo "  RKV_RUN_SELF_TEST=1       等价于 --self-test"
}

MODE="all"
USE_OLD_ENV="false"
SKIP_SWITCH_ENV="false"
RUN_SELF_TEST="false"

for arg in "$@"; do
    case "$arg" in
        -q|--quick)
            MODE="quick"
            ;;
        --all)
            MODE="all"
            ;;
        -old)
            USE_OLD_ENV="true"
            ;;
        --skip-env)
            SKIP_SWITCH_ENV="true"
            ;;
        --self-test)
            RUN_SELF_TEST="true"
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "未知参数: $arg" >&2
            show_help
            exit 1
            ;;
    esac
done

SWITCH_ENV_SCRIPT="$HOME/switch_env.sh"
if [ "$SKIP_SWITCH_ENV" == "true" ] || [ "${RKV_SKIP_SWITCH_ENV:-0}" == "1" ]; then
    echo "跳过 switch_env.sh"
elif [ -f "$SWITCH_ENV_SCRIPT" ]; then
    echo "执行 switch_env.sh control ..."
    cd "$SCRIPT_DIR" && "$SWITCH_ENV_SCRIPT" control
    if [ "$USE_OLD_ENV" == "true" ]; then
        echo "使用旧版环境"
    else
        echo "使用新版环境"
    fi
else
    echo "未检测到 switch_env.sh，跳过"
fi

if [ "$MODE" == "quick" ]; then
    BUILD_SCRIPT="$SCRIPT_DIR/build.sh"
else
    BUILD_SCRIPT="$SCRIPT_DIR/all_rebuild.sh"
fi

COLLECT_DEP_SCRIPT="$SCRIPT_DIR/scripts/collect_dep.sh"
COPY_FILES_FOR_RELEASE_SCRIPT="$SCRIPT_DIR/scripts/copy_files_for_release.sh"
STAGE_DIR="$SCRIPT_DIR/robot_kinematic_viewer_release"
TARGET_RELEASE_DIR="${RKV_RELEASE_TARGET_DIR:-$SCRIPT_DIR/../../robot_kinematic_viewer_release}"

if [ ! -x "$BUILD_SCRIPT" ] && [ -f "$BUILD_SCRIPT" ]; then
    chmod +x "$BUILD_SCRIPT"
fi
for s in "$COLLECT_DEP_SCRIPT" "$COPY_FILES_FOR_RELEASE_SCRIPT"; do
    if [ -f "$s" ] && [ ! -x "$s" ]; then
        chmod +x "$s"
    fi
done

echo "========== 开始执行构建脚本 =========="
if [ -f "$BUILD_SCRIPT" ]; then
    cd "$SCRIPT_DIR" && bash "$BUILD_SCRIPT"
else
    echo "❌ 错误: 脚本不存在: $BUILD_SCRIPT" >&2
    exit 1
fi

echo "========== 开始收集依赖（staging: $STAGE_DIR）=========="
if [ -f "$COLLECT_DEP_SCRIPT" ]; then
    cd "$SCRIPT_DIR" && bash "$COLLECT_DEP_SCRIPT"
else
    echo "❌ 错误: 脚本不存在: $COLLECT_DEP_SCRIPT" >&2
    exit 1
fi

echo "========== 开始复制发布目录 =========="
if [ -f "$COPY_FILES_FOR_RELEASE_SCRIPT" ]; then
    cd "$SCRIPT_DIR" && bash "$COPY_FILES_FOR_RELEASE_SCRIPT"
else
    echo "❌ 错误: 脚本不存在: $COPY_FILES_FOR_RELEASE_SCRIPT" >&2
    exit 1
fi

if [ "$RUN_SELF_TEST" == "true" ] || [ "${RKV_RUN_SELF_TEST:-0}" == "1" ]; then
    echo "========== 执行发布包自检 =========="
    if [ -x "$TARGET_RELEASE_DIR/run.sh" ]; then
        "$TARGET_RELEASE_DIR/run.sh" --self-test
        echo "✅ 发布包自检通过"
    else
        echo "❌ 错误: 找不到可执行自检脚本: $TARGET_RELEASE_DIR/run.sh" >&2
        exit 1
    fi
fi

if [ "${RKV_KEEP_STAGE_DIR:-0}" == "1" ]; then
    echo "========== 保留中间 staging 目录 =========="
    echo "已保留: $STAGE_DIR"
else
    echo "========== 清理中间 staging 目录 =========="
    rm -rf "$STAGE_DIR"
fi

echo "✅ ✅ ✅ 全流程完成！robot_kinematic_viewer 发布包已就绪: $TARGET_RELEASE_DIR"
