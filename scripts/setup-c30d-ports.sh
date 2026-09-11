#!/usr/bin/env bash
# setup-c30d-ports.sh —— 按板载 CH9102 的 USB 序列号解析/固定两个串口
#
# 板上有两颗 CH9102（VID 1a86），ttyUSB/ttyACM 编号随插拔顺序漂移，
# 但 USB 序列号固化在芯片里、跟板走，是最稳的锚点：
#   USB1（USART1，烧录口 + 调试 shell）  -> c30d-flash
#   USB3（USART3，帧协议口）             -> c30d-proto
#
# 用法：
#   root：安装 udev 规则，固定出 /dev/c30d-flash 与 /dev/c30d-proto
#   普通用户：打印解析结果（供 chassis_host --serial/--flash-dev 与
#             flash.py 使用；WSL 无 root udev 时即此模式）
# 换板/换芯片时序列号会变，用环境变量覆盖：
#   C30D_FLASH_SERIAL / C30D_PROTO_SERIAL
set -eu

FLASH_SERIAL="${C30D_FLASH_SERIAL:-5C2C059293}"
PROTO_SERIAL="${C30D_PROTO_SERIAL:-5C2C059299}"
RULE_FILE=/etc/udev/rules.d/99-c30d-ports.rules

resolve() {  # $1=序列号；成功输出 /dev/ttyXXX
  local d tty dev
  for d in /sys/class/tty/*/device; do
    tty=${d%/device}; tty=${tty#/sys/class/tty/}
    case "$tty" in ttyACM*|ttyUSB*) ;; *) continue ;; esac
    dev=$(readlink -f "$d" 2>/dev/null) || continue
    dev=$(dirname "$dev")
    [ -f "$dev/serial" ] || continue
    if [ "$(cat "$dev/serial")" = "$1" ]; then
      echo "/dev/$tty"
      return 0
    fi
  done
  return 1
}

FLASH_DEV=$(resolve "$FLASH_SERIAL" || true)
PROTO_DEV=$(resolve "$PROTO_SERIAL" || true)

if [ -z "$FLASH_DEV" ] || [ -z "$PROTO_DEV" ]; then
  echo "未找全 C30D 串口（flash=${FLASH_DEV:-无} proto=${PROTO_DEV:-无}）"
  echo "检查：线是否都插好 / usbipd 是否已 attach / 序列号是否变化"
  exit 1
fi

echo "flash ($FLASH_SERIAL) -> $FLASH_DEV"
echo "proto ($PROTO_SERIAL) -> $PROTO_DEV"

if [ "$(id -u)" = 0 ]; then
  cat > "$RULE_FILE" <<EOF
# C30D 板载双 CH9102 固定名（本脚本生成；序列号见芯片本体）
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="$FLASH_SERIAL", SYMLINK+="c30d-flash"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="$PROTO_SERIAL", SYMLINK+="c30d-proto"
EOF
  udevadm control --reload-rules
  udevadm trigger --subsystem-match=tty
  echo "已安装 udev 规则 $RULE_FILE：/dev/c30d-flash /dev/c30d-proto"
else
  echo "无 root，不装 udev 规则。启动示例："
  echo "  chassis_host --serial $PROTO_DEV --flash-dev $FLASH_DEV \\"
  echo "    --flash-script <C30D_Chassis>/host_test/flash.py --firmware-dir ~/firmware"
  echo "提示：树莓派上建议 systemctl disable ModemManager（防其开串口误拉 DTR/RTS 复位板子）"
fi
