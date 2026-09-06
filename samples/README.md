# samples —— MATLAB 客户端示例

面向 MATLAB 用户的 `chassis_host` 控制例程：通过 TCP 文本协议下发指令、
订阅 JSON 遥测并采集到工作区变量中绘图/分析。只用**基础 MATLAB**
（R2020b 及以上，用到 `tcpclient`/`writeline`）。

## 文件

| 文件 | 内容 |
|---|---|
| `ChassisClient.m` | TCP 文本协议客户端类：指令下发 + 应答等待 + JSON 遥测解析入缓冲 |
| `demo_odom.m` | 完整演示：连接 → 等链路 READY → 低速直行 3s → 刹车 → 绘制里程计轨迹与速度曲线 |

## 快速上手

把 `samples/` 加入 MATLAB 路径（`cd` 过去或 `addpath`），宿主已在运行
（见 `chassis-host/README.md`「构建与运行」一节）后：

```matlab
c = ChassisClient("192.168.1.10");   % 树莓派 IP；本机联调填 127.0.0.1
c.connect();
c.cmd("watch on");                   % 订阅 JSON 遥测
c.startTelemetry();                  % 后台定时器接收遥测（默认 20Hz 轮询）
c.waitReady();                       % 等链路 READY（宿主自动建链 + 使能）

c.drive(0.15, 0, 0);                 % 低速前进；宿主 10Hz 重发，只需发一次
pause(3);
c.stop();

o = c.getOdom();
plot(o.x, o.y, "-o"); axis equal; grid on;

c.disconnect();                      % 断开前已 stop；宿主对断开的指令来源也会自动停车
```

或直接跑 `demo_odom`。

## ChassisClient 接口速查

方法：

| 方法 | 说明 |
|---|---|
| `connect(timeout)` | 建立连接，消费欢迎行与首条 con 事件 |
| `cmd(line, timeout)` | 下发任意一行指令，阻塞等 `ok`/`err` 应答，返回 `[isok, ack]`；`ping` 以 `pong` 行为应答 |
| `drive(vx, vy, wz)` | 车体速度 `v`（m/s、rad/s），省略参数补 0 |
| `setRpm(r1, r2, r3)` | 三电机目标转速 `rpm` |
| `stop()` | 刹车并停止重发 |
| `ping(timeout)` | 测板端 RTT，返回 `[isok, rttms]` |
| `startTelemetry(rateHz)` / `stopTelemetry()` | 后台定时器收遥测（默认 20Hz 轮询排空 socket） |
| `waitReady(timeout)` | 等待 `STATUS.con=1 && en=1`（宿主建链后自动使能） |
| `getOdom/getEnc/getImu/getTelem` | 各遥测缓冲的**列式视图**（struct of arrays，N×3 矩阵），直接可 `plot` |
| `clearTelemetry()` | 清空全部缓冲 |
| `pumpLines()` | 手动排空一次收流（不想用定时器时在循环里自己调） |
| `disconnect()` | 断开（先补发一次 `stop`） |

属性（只读）：

| 属性 | 说明 |
|---|---|
| `Odom` / `Enc` / `Imu` / `Telem` | 遥测缓冲（cell of struct，逐拍追加，含秒级时间戳 `t`） |
| `LatestStatus` | 最近一拍 `status`：`uptime/imu_ok/en/mode/con` |
| `Events` | `con`/`serial`/`board` 事件流（原始 JSON struct 在 `data` 字段） |
| `Messages` | 非 JSON 行（欢迎语、`ok`/`err`/`pong`、`help` 输出），排查用 |

## 与协议的关系（读一遍少踩坑）

- **链路保活不归 MATLAB 管**：CON 建链/看门狗、速度指令 10Hz 重发、
  指令门控全部由 `chassis_host` 完成（`docs/protocol-serial.md`）。MATLAB 端只发
  一次 `v`，宿主持续重发；宿主自己喂板侧看门狗。
- **断开即停车**：`v`/`rpm` 指令来源的 TCP 连接断开，宿主立即
  CMD_STOP。`disconnect()` 里仍会显式补发一次 `stop`，双保险。
- **遥测要订阅**：`watch on` 只对本连接生效；`odom` 默认 10Hz、
  `enc/imu/telem/status` 默认 1Hz（`--odom-hz`/`--slow-hz` 可调）。
  `con`/`serial` 事件推送给所有连接。
- **多控制端**：多个 MATLAB/nc 会话可同时 `watch`；但速度指令"后到
  覆盖"，不要两个会话同时发 `v`。
- **帮助输出**：`cmd("help")` 只等到首行 `ok`，后续行进入 `Messages`。

## 本机无车联调

用 `tools/mock_board` 模拟下位机即可在纯 PC 上跑通（`docs/testing.md`）：

```bash
./build/host/mock_board --pty --link /tmp/mock_tty     # 终端 1
./build/host/chassis_host --serial /tmp/mock_tty       # 终端 2
```

MATLAB 侧 `ChassisClient("127.0.0.1")` 直接连。

## 已知限制

- 遥测定时器回调在 MATLAB 主线程调度：长阻塞计算期间不收遥测，数据
  会在 socket 缓冲里积压，计算结束后由下一次轮询补上（`t` 为收包时刻，
  不是严格到达时刻）。
- `tcpclient` 半行残包需自缓冲，本类已处理；但请不要多个对象连同一
  端口后互相拷贝句柄属性。
