#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""selftest.py —— chassis-host 端到端集成自测（无硬件）

拓扑：mock_board(pty) ←串口帧协议→ chassis_host ←TCP 文本协议→ 本脚本

覆盖场景：
  1. CON 建链 READY + 自动下发 CMD_EN
  2. watch 订阅后收到 JSON 遥测（odom/enc/imu/status）
  3. v 指令 → mock 转速爬升（odom vx > 0.1）
  4. ping RTT / stat 统计
  5. TCP 控制端断开 → 宿主自动 CMD_STOP → mock 停车
  6. mock 退出 → 宿主上报串口 down；mock 重启（同符号链接）→ 宿主自动重开
     串口并重新建链（设备消失自愈）
  7. en 0 → 拆链失能且不被自动补发覆盖；en 1 → 恢复
  8. SIGTERM 优雅退出，日志文件已生成

用法: python3 selftest.py [build 目录]（默认 build/host，二进制所在目录）
"""

import json
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time

HOST_BIN = "chassis_host"
MOCK_BIN = "mock_board"
LINK_PATH = "/tmp/mhost_selftest_tty"

g_checks = 0
g_failures = 0


def check(cond, msg):
    global g_checks, g_failures
    g_checks += 1
    if cond:
        print("  ok  %s" % msg)
    else:
        g_failures += 1
        print("  FAIL %s" % msg)


class TcpClient:
    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.buf = b""
        self.pending = []  # 已收到但未被任何 wait 消费的行（不丢弃）

    def send(self, line):
        self.sock.sendall((line + "\n").encode())

    def pump(self):
        """非阻塞收新数据并入 pending"""
        self.sock.setblocking(False)
        try:
            while True:
                data = self.sock.recv(65536)
                if not data:
                    break
                self.buf += data
                while b"\n" in self.buf:
                    line, self.buf = self.buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").strip()
                    if text:
                        self.pending.append(text)
        except (BlockingIOError, OSError):
            pass
        self.sock.setblocking(True)

    def wait_for(self, pred, timeout, desc):
        """轮询等待 pending 中出现满足 pred 的行（保留未匹配行不丢）"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.pump()
            for i, line in enumerate(self.pending):
                if pred(line):
                    del self.pending[: i + 1]  # 连同匹配行一起移除更早的已检查行
                    return line
            time.sleep(0.05)
        return None

    def wait_json(self, t, pred, timeout, desc):
        def p(line):
            if not line.startswith("{"):
                return False
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                return False
            return obj.get("t") == t and pred(obj)
        return self.wait_for(p, timeout, desc)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def start_mock(build_dir):
    """启动 mock_board；stdout 走临时文件（管道有写满阻塞风险），从中取 pty 路径"""
    outf = tempfile.TemporaryFile()
    proc = subprocess.Popen([os.path.join(build_dir, MOCK_BIN), "--pty", "--link", LINK_PATH],
                            stdout=outf, stderr=subprocess.STDOUT)
    for _ in range(100):
        outf.seek(0)
        text = outf.read().decode("utf-8", "replace")
        m = re.search(r"pty: (\S+)", text)
        if m:
            return proc, m.group(1), outf
        if proc.poll() is not None:
            raise RuntimeError("mock_board 提前退出: %r" % text)
        time.sleep(0.05)
    raise RuntimeError("mock_board 未输出 pty 路径")


def main():
    build_dir = sys.argv[1] if len(sys.argv) > 1 else "build/host"
    tmp = tempfile.mkdtemp(prefix="mhost_selftest_")
    log_dir = os.path.join(tmp, "log")
    port = free_port()

    print("== chassis-host 集成自测 ==")
    print("build=%s port=%d log=%s" % (build_dir, port, log_dir))

    mock, _pty, mock_out = start_mock(build_dir)
    host = subprocess.Popen([
        os.path.join(build_dir, HOST_BIN),
        "--serial", LINK_PATH,
        "--listen", "127.0.0.1:%d" % port,
        "--log-dir", log_dir,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    try:
        # 等待 TCP 就绪
        cli = None
        for _ in range(50):
            try:
                cli = TcpClient(port)
                break
            except OSError:
                if host.poll() is not None:
                    raise RuntimeError("chassis_host 提前退出")
                time.sleep(0.1)
        check(cli is not None, "TCP 连接建立")

        # 先订阅遥测（后续所有断言都依赖 JSON 流）
        cli.send("watch on")
        got = cli.wait_for(lambda l: "watch=on" in l, 3, "watch on 应答")
        check(got is not None, "watch on 应答")

        # 1) CON 建链 + 自动 EN
        got = cli.wait_json("con", lambda o: o.get("state") == "READY", 8, "CON 建链 READY")
        check(got is not None, "CON 建链 READY 广播")
        got = cli.wait_json("status", lambda o: o.get("en") == 1, 5, "自动 CMD_EN")
        check(got is not None, "READY 后自动下发 CMD_EN（STATUS en=1）")

        # 2) 遥测内容
        got = cli.wait_json("odom", lambda o: True, 3, "odom JSON")
        check(got is not None, "收到 odom JSON 遥测")
        got = cli.wait_json("enc", lambda o: True, 3, "enc JSON")
        check(got is not None, "收到 enc JSON 遥测")
        got = cli.wait_json("imu", lambda o: True, 3, "imu JSON")
        check(got is not None, "收到 imu JSON 遥测")

        # 3) 速度指令 → mock 转速爬升
        cli.send("v 0.2 0 0")
        got = cli.wait_for(lambda l: l.startswith("ok twist"), 3, "v 指令应答")
        check(got is not None, "v 指令应答")
        got = cli.wait_json("odom", lambda o: o.get("vx", 0) > 0.1, 5, "vx 爬升")
        check(got is not None, "mock 电机响应 vx>0.1（10Hz 重发喂活 500ms 超时）")

        # 4) ping / stat
        cli.send("ping")
        got = cli.wait_for(lambda l: l.startswith("pong rtt="), 3, "pong RTT")
        check(got is not None, "ping RTT 应答")
        cli.send("stat")
        got = cli.wait_for(
            lambda l: l.startswith("ok serial=up con=CONNECTED/READY") and "crc=0" in l,
            3, "stat")
        check(got is not None, "stat: 串口 up / CON READY / 零 CRC 错")

        # 5) 控制端断开 → 自动停车
        cli.close()
        cli2 = TcpClient(port)
        cli2.send("watch on")
        cli2.wait_json("con", lambda o: True, 3, "订阅")
        got = cli2.wait_json("odom", lambda o: o.get("vx", 1) < 0.05, 5, "断开自动停车")
        check(got is not None, "TCP 控制端断开 → 自动 CMD_STOP 停车")

        # 6) 设备消失自愈：杀 mock → 串口 down；重启 mock → 自动重开 + 重新建链
        mock.send_signal(signal.SIGTERM)
        got = cli2.wait_json("serial", lambda o: o.get("up") is False, 8, "串口 down 广播")
        check(got is not None, "mock 退出 → serial up=false 广播")
        mock.wait(timeout=5)
        mock, _, mock_out = start_mock(build_dir)
        got = cli2.wait_json("serial", lambda o: o.get("up") is True, 10, "串口重开")
        check(got is not None, "mock 重启（同符号链接）→ 宿主自动重开串口")
        got = cli2.wait_json("con", lambda o: o.get("state") == "READY", 10, "重新建链")
        check(got is not None, "串口恢复后 CON 重新建链 READY")
        time.sleep(1.5)  # 等 ping 节拍稳定，避开 en0 与重连自愈 EN 的竞态窗口

        # 7) en 0 / en 1
        cli2.send("en 0")
        got = cli2.wait_for(lambda l: l.startswith("ok en=0"), 3, "en 0 应答")
        check(got is not None, "en 0 应答")
        got = cli2.wait_json("status", lambda o: o.get("en") == 0, 3, "en=0 生效")
        check(got is not None, "en 0 生效（STATUS en=0）")
        got = cli2.wait_json("status", lambda o: o.get("en") == 1, 2.5, "不应自动补发")
        check(got is None, "拆链后 READY 自愈补发不再覆盖 en=0")
        cli2.send("en 1")
        got = cli2.wait_json("status", lambda o: o.get("en") == 1, 6, "en 1 恢复")
        check(got is not None, "en 1 → 重新建链并使能（STATUS en=1）")

        # 8) 收尾
        cli2.send("stop")
        cli2.wait_for(lambda l: l.startswith("ok stop"), 3, "stop 应答")
        cli2.close()
    finally:
        # 8) 优雅退出检查
        host.send_signal(signal.SIGTERM)
        try:
            rc = host.wait(timeout=5)
        except subprocess.TimeoutExpired:
            host.kill()
            rc = -1
        check(rc == 0, "SIGTERM 优雅退出（rc=0）")
        logs = [f for f in os.listdir(log_dir)] if os.path.isdir(log_dir) else []
        check(any(f.startswith("chassis_host.") and f.endswith(".log") for f in logs),
              "双缓冲日志文件已生成: %s" % logs)
        if mock.poll() is None:
            mock.terminate()
            mock.wait(timeout=5)

        if g_failures:
            print("---- chassis_host 日志（失败转储）----")
            for f in os.listdir(log_dir) if os.path.isdir(log_dir) else []:
                p = os.path.join(log_dir, f)
                if os.path.getsize(p) < 2_000_000:
                    with open(p, "rb") as fh:
                        sys.stdout.write(fh.read().decode("utf-8", "replace"))

    print("selftest: %d checks, %d failures" % (g_checks, g_failures))
    return 0 if g_failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
