#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""client.py —— chassis_host 的轻量 TCP 测试客户端（纯标准库）

交互模式（仿 host_test/bridge_cli.py 风格）:
    python3 client.py --host 127.0.0.1 --port 9000
    > v 0.2 0 0          # 车体速度指令
    > watch on           # 订阅 JSON 遥测
    > stat / con / ping / stop / q

单发模式（脚本用）:
    python3 client.py --host 127.0.0.1 --port 9000 --cmd "v 0.2 0 0" [--wait 1.0]
"""

import argparse
import socket
import sys
import threading
import time

HELP = """命令（与 TCP 文本协议一致）:
  v <vx> [vy] [wz]    车体速度 m/s、rad/s
  rpm <r1> [r2] [r3]  三电机目标转速 rpm
  stop                刹车
  en <0|1>            电机使能
  pid <kp> <ki> <kd>  设置 PID
  sign <s1> <s2> <s3> 编码器方向校正
  ping                测板端 RTT
  watch [on|off]      订阅/退订遥测
  stat | con | help   统计 / CON 状态 / 帮助
  q                   退出客户端（连接断开会触发宿主自动停车）"""


def reader(sock: socket.socket) -> None:
    """后台线程：打印宿主下发的所有行（响应 ok/err + JSON 遥测）"""
    buf = b""
    try:
        while True:
            data = sock.recv(4096)
            if not data:
                print("[连接已断开]")
                sys.exit(0)
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").rstrip("\r")
                if text:
                    print(text, flush=True)
    except (ConnectionResetError, OSError):
        print("[连接已断开]")
        sys.exit(0)


def main() -> int:
    ap = argparse.ArgumentParser(description="chassis_host TCP 测试客户端")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--cmd", default=None, help="单发模式：发送一行后等待 --wait 秒退出")
    ap.add_argument("--wait", type=float, default=1.0)
    args = ap.parse_args()

    sock = socket.create_connection((args.host, args.port), timeout=5)
    sock.settimeout(None)

    t = threading.Thread(target=reader, args=(sock,), daemon=True)
    t.start()

    if args.cmd is not None:
        sock.sendall((args.cmd + "\n").encode())
        time.sleep(args.wait)
        sock.close()
        return 0

    print(HELP)
    try:
        while True:
            line = input("> ").strip()
            if line in ("q", "quit", "exit"):
                break
            if not line:
                continue
            sock.sendall((line + "\n").encode())
    except (KeyboardInterrupt, EOFError):
        pass
    sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
