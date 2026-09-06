#!/usr/bin/env bash
# build_muduo.sh —— 构建 muduo 网络库并安装到本工程 build/muduo-install
# （lib + include），供本工程 CMake 直接链接。
#
# 用法（Linux / WSL2 / 树莓派）:
#   bash scripts/build_muduo.sh [muduo 源码目录]
#
# 源码目录按优先级解析：位置参数 > 环境变量 MUDUO_SRC > 自动查找
#   ./muduo         克隆到仓库内（不推荐，会污染 git status）
#   ../muduo        与本仓库并列 git clone 的 muduo（推荐布局）
# 独立使用请先获取源码:
#   git clone https://github.com/chenshuo/muduo   （或其 Gitee 镜像）
#
# 可选环境变量:
#   MUDUO_SRC         muduo 源码目录（与位置参数等价）
#   MUDUO_BUILD_DIR   muduo 编译目录（默认 build/muduo-build；drvfs 慢时可指到
#                     WSL 原生路径加速，如 MUDUO_BUILD_DIR=~/tmp/muduo-build）
#   MUDUO_INSTALL_DIR 安装前缀（默认 build/muduo-install）
#
# 依赖: g++ / cmake / Boost 头文件（sudo apt install libboost-dev）
# 兼容性: CMake 4.x 移除了 <3.5 兼容，脚本统一传
#         -DCMAKE_POLICY_VERSION_MINIMUM=3.5（旧版 CMake 忽略该变量，无害）。
#         GCC 13+ 下 muduo 的 -Werror 可能报新告警，失败时自动把源码复制到
#         build/muduo-src 剔除 -Werror 后重试（生成的副本属构建产物）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(cd "$ROOT/.." && pwd)"
BUILD_DIR="${MUDUO_BUILD_DIR:-$ROOT/build/muduo-build}"
INSTALL_DIR="${MUDUO_INSTALL_DIR:-$ROOT/build/muduo-install}"

MUDUO_SRC="${MUDUO_SRC:-${1:-}}"
if [ -z "$MUDUO_SRC" ]; then
  for cand in "$ROOT/muduo" "$WS_ROOT/muduo"; do
    if [ -f "$cand/CMakeLists.txt" ]; then
      MUDUO_SRC="$cand"
      break
    fi
  done
fi

if [ -z "$MUDUO_SRC" ]; then
  echo "错误: 未找到 muduo 源码，已尝试: ./muduo ../muduo" >&2
  echo "      请先 git clone https://github.com/chenshuo/muduo（或其 Gitee 镜像），" >&2
  echo "      与本仓库并列放置；或把源码目录作为参数 / 环境变量 MUDUO_SRC 传入" >&2
  exit 1
fi
echo "== muduo 源码: $MUDUO_SRC =="

if [ ! -f "$MUDUO_SRC/CMakeLists.txt" ]; then
  echo "错误: 找不到 muduo 源码 $MUDUO_SRC" >&2
  echo "      请先 git clone https://github.com/chenshuo/muduo（或其 Gitee 镜像），" >&2
  echo "      并把源码目录作为参数传给本脚本" >&2
  exit 1
fi
if [ ! -f /usr/include/boost/version.hpp ]; then
  echo "错误: 缺少 Boost 头文件，请先执行: sudo apt install libboost-dev" >&2
  exit 1
fi
case "$BUILD_DIR" in
  /mnt/*) echo "[提示] 构建目录在 drvfs(/mnt) 上编译较慢，可用 MUDUO_BUILD_DIR=~/tmp/... 加速" ;;
esac

build_and_install() { # $1=源码目录 $2=构建目录
  cmake -S "$1" -B "$2" \
    -DCMAKE_BUILD_TYPE=Release \
    -DMUDUO_BUILD_EXAMPLES=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
  cmake --build "$2" -j"$(nproc)"
  cmake --install "$2"
}

rm -rf "$INSTALL_DIR"
if build_and_install "$MUDUO_SRC" "$BUILD_DIR"; then
  echo "== muduo 构建安装完成: $INSTALL_DIR =="
  ls "$INSTALL_DIR/lib"
else
  echo "[回退] 首次编译失败（多为新 GCC 下 -Werror 报错），复制源码剔除 -Werror 重试…" >&2
  FALLBACK_SRC="$ROOT/build/muduo-src"
  rm -rf "$FALLBACK_SRC" "$BUILD_DIR"
  mkdir -p "$FALLBACK_SRC"
  cp -r "$MUDUO_SRC/." "$FALLBACK_SRC/"
  sed -i 's/-Werror//' "$FALLBACK_SRC/CMakeLists.txt"
  build_and_install "$FALLBACK_SRC" "$BUILD_DIR"
  echo "== muduo 构建安装完成（副本已剔除 -Werror）: $INSTALL_DIR =="
  ls "$INSTALL_DIR/lib"
fi
