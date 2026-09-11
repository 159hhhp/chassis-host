#!/usr/bin/env bash
# run_selftest.sh —— 一键构建并运行全部测试（单测 + 日志基准 + 端到端集成自测）
# 用法（WSL2 / Linux）: bash scripts/run_selftest.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-$ROOT/build/host}"

cd "$ROOT"
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo "== 单元测试 =="
"$BUILD_DIR/test_framecodec"
"$BUILD_DIR/test_consm"
"$BUILD_DIR/test_otaflasher"

echo "== 日志基准（4 线程 x 50000 行）=="
"$BUILD_DIR/log_bench" 4 50000

echo "== 端到端集成自测 =="
python3 tools/selftest.py "$BUILD_DIR"
