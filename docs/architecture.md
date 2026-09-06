# 架构与实现

> 返回 [README](../README.md)。本文说明总体架构、单 Reactor 零锁模型，
> 以及把串口挂进 EventLoop 的实现要点。协议细节见
> [protocol-tcp.md](protocol-tcp.md) 与 [protocol-serial.md](protocol-serial.md)。

```
 TCP client(s)                  on-board Linux (Pi / WSL2)              STM32
+-------------+              +----------------------------+          +---------+
| nc /        |  text cmds / | TcpServer (muduo) ---+     |  binary  | C30D    |
| client.py   | <----------- | ack "ok"/"err"       |     |  frames  | firmware|
+-------------+              |                      v     |          | (USART3 |
                             |         ChassisHost        | <------- | CH9102F |
                             | (gating / 10Hz repeat /    |  ENC/IMU/| -> USB3)|
                             |  watchdogs / JSON out)     |  ODOM/CON+---------+
                             | SerialPort(Channel->epoll) |
                             +------- one EventLoop ------+
                                 |
                                 | log/ async double-buffer
                                 | logging (own bg thread)
```

单 Reactor 线程（`TcpServer::setThreadNum(0)`）：TCP 收发、串口收发、
链路状态机 tick、指令重发、看门狗检查全部在一个事件循环里串行执行，
**零锁**。唯一的其他线程是日志后台落盘线程。

## 串口挂载到 EventLoop 的实现

muduo 的 `Channel` 只绑定 `(EventLoop*, fd)`，Poller 层（`EPollPoller`）
是纯 `epoll_wait`/`epoll_ctl` 转发，对 fd 的类型没有任何 socket 假设，
因此 tty 这类可 poll 设备可以直接挂进 Reactor，与 TCP 连接共用一次
`epoll_wait`——无需独立的桥接线程。`serial/SerialPort` 的实现模式：

- 打开：`open(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC)` + termios
  原始 8N1（`cfmakeraw`，无流控）；不使用 muduo 的 `Socket/SocketsOps`
  （TCP 专用封装）；
- 读：水平触发 + 循环 `read` 到 `EAGAIN`，一次唤醒排空；tty 须配置
  `VMIN=1`（`O_NONBLOCK` 下 `VMIN=0` 的无数据 `read` 返回 0，无法与
  EOF 区分）；
- 写：仿 `TcpConnection` —— 缓冲为空先直写，`EAGAIN` 剩余进发送缓冲并
  `enableWriting()`，`POLLOUT` 写完 `disableWriting()`；
- 生命周期：`disableAll()` + `remove()` 后才 `close(fd)`；全部 Channel
  操作经 `loop->runInLoop()` 保证线程约束；
- 掉线自愈：`POLLHUP`/`POLLERR`/读写致命错误 → 拆除 Channel 并关闭 fd，
  上层定时器周期重试打开，恢复后自动重新建链。
