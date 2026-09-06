# chassis-host —— 基于 muduo 的轻量级机器人底盘上位机

基于 [muduo 网络库](https://github.com/chenshuo/muduo)（Reactor 模式）实现的
Linux 侧上位机：TCP 服务器接收操控指令，经串口以自定义二进制帧协议转发给
STM32 下位机，同时把下位机遥测降采样为 JSON 行推送给 TCP 客户端。
无 ROS 依赖、单二进制、`nc` 即可操控，适合作为小型底盘车的车端常驻控制服务。

**特性**

- **串口与 TCP 共用同一个 EventLoop**：tty 设备直接挂进 Reactor 的 epoll，
  与 TCP 收发在单线程事件循环内串行处理，全链路无锁（见
  [docs/architecture.md](docs/architecture.md)）
- **TCP 文本指令 + JSON 遥测**：telnet/netcat 交互即可操控（见
  [docs/protocol-tcp.md](docs/protocol-tcp.md)）
- **链路保活与多层失效保护**：CON 连接状态机喂板侧看门狗、速度指令周期
  重发、控制端掉线自动停车、串口设备热插拔自愈（见
  [docs/protocol-serial.md](docs/protocol-serial.md)）
- **自研双缓冲异步日志库**：参考 muduo AsyncLogging 设计实现，前端写日志
  永不阻塞在磁盘 I/O（见 [docs/logging.md](docs/logging.md)）
- 完整的离线测试链：协议/状态机单元测试 + 伪终端模拟固件的端到端自测
  （见 [docs/testing.md](docs/testing.md)）

## 1. 快速体验

宿主运行后，任意 TCP 客户端均可操控：

```
$ nc 192.168.1.10 9000
v 0.2 0 0.5
ok twist vx=0.200 vy=0.000 wz=0.500（已下发，10Hz 重发中）
watch on
{"t":"odom","vx":0.195,"vy":0.000,"wz":0.481,"x":0.112,"y":0.000,"yaw":0.049}
{"t":"status","uptime":37,"imu_ok":1,"en":1,"mode":1,"con":1}
ping
pong rtt=3.0ms
stop
ok stop
```

## 2. 构建与运行

依赖：g++、CMake ≥ 3.13、Boost 头文件（`sudo apt install libboost-dev`）、
Python3（仅测试工具）。Linux 原生 / WSL2 / 树莓派均可。

```bash
# 1) 获取 muduo 源码（BSD License），与本仓库并列放置
git clone https://github.com/chenshuo/muduo    # 或其 Gitee 镜像

# 2) 构建 muduo 静态库并安装到 build/muduo-install
bash scripts/build_muduo.sh    # 自动查找 ./muduo；或用环境变量 MUDUO_SRC 指定

# 3) 构建本工程（自动探测 build/muduo-install 或 /usr/local）
cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Release
cmake --build build/host -j

# 4) 运行
./build/host/chassis_host --serial /dev/ttyUSB0 --listen 0.0.0.0:9000
```

一键自测（构建 + 全部测试，详见 [docs/testing.md](docs/testing.md)）：

```bash
bash scripts/run_selftest.sh
```

CLI 参数：`--serial`（默认 /dev/ttyUSB0）、`--baud`（115200，8N1）、
`--listen`（0.0.0.0:9000）、`--log-dir`（默认 log；空串=stdout）、
`--log-level`（trace/debug/info/warn/error/fatal）、`--repeat-hz`（10）、
`--odom-hz`（10）、`--slow-hz`（1）、`--con-retry-ms`（2000）、
`--rx-timeout-ms`（2000）、`--en-resend-ms`（1000）。

串口侧只需下位机实现 [docs/protocol-serial.md](docs/protocol-serial.md)
的帧协议（任意 USB-TTL 适配器均可，波特率 115200-8N1）；程序对设备中途
消失/重现有自愈能力。

## 3. 目录结构

```
chassis-host/
├── README.md            本文件
├── docs/                设计文档（架构 / 协议 / 日志 / 测试）
├── CMakeLists.txt
├── log/                 双缓冲异步日志库（无 muduo 依赖，可独立复用）
├── proto/               串口帧协议编解码（纯 C++17）
├── link/                CON 链路保活状态机（虚拟时钟可单测）
├── serial/              SerialPort：tty 挂 muduo epoll
├── app/                 ChassisHost（组装/门控/重发/看门狗/JSON）+ main
├── tests/               单元测试、日志基准
├── tools/               mock_board（pty 模拟下位机）、client.py、selftest.py
├── samples/             MATLAB 客户端示例（ChassisClient 类 + demo_odom，见 samples/README.md）
├── scripts/             build_muduo.sh、run_selftest.sh
└── build/               构建产物（muduo-install / host 等）
```

## 4. 文档

| 文档 | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | 总体架构、单 Reactor 零锁模型、串口挂 EventLoop 的实现要点 |
| [docs/protocol-tcp.md](docs/protocol-tcp.md) | TCP 文本协议：指令表、应答格式、JSON 遥测、多连接语义 |
| [docs/protocol-serial.md](docs/protocol-serial.md) | 串口帧协议、CON 链路保活状态机、多层失效保护 |
| [docs/logging.md](docs/logging.md) | 双缓冲异步日志库的四层结构与实测吞吐 |
| [docs/testing.md](docs/testing.md) | 测试链总览与 mock_board 人工联调 |

## 5. 已知限制

- TCP 明文无鉴权，面向实验室内网/有线场景；对外暴露端口需自行加防护。
- 多控制端并发写入为"后到覆盖"语义，无仲裁。
- 运动学几何参数（轮径/轮距/安装角）由下位机固件定义，本程序不做标定。
- 日志按进程生命周期滚动，不做压缩/清理，`log/` 目录需人工清理。

## 6. 致谢

本项目的网络模型（Reactor、one loop per thread）与双缓冲日志设计学习自
[陈硕](https://github.com/chenshuo)的开源项目
[muduo 网络库](https://github.com/chenshuo/muduo)（BSD License）；
`serial/SerialPort` 对裸文件描述符的挂载方式、`log/` 的实现思路均参考了
muduo 的源码与《Linux 多线程服务端编程：使用 muduo C++ 网络库》一书。
muduo 以静态库形式在构建期引入，本仓库不包含其源码副本。
