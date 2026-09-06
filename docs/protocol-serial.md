# 串口帧协议与链路看门狗

> 返回 [README](../README.md)。本仓库 `proto/` 为该协议的完整 C++17
> 实现（编解码 + 流式解帧状态机），不依赖 muduo，可单独复用；据此亦可
> 自行实现下位机侧。

## 帧格式（小端）

```
AA 55 | type(1B) | len(1B) | seq(1B) | payload(len B) | crc16_lo crc16_hi
```

- `len` ≤ 32，帧总开销 7 字节；
- CRC16-MODBUS（多项式 0xA001，初值 0xFFFF），覆盖 `type + len + seq +
  payload`，不含帧头；标准测试向量 `"123456789"` → `0x4B37`；
- 解帧容错：坏帧（CRC 错）整帧丢弃并重新同步；`len > 32` 的伪帧头跳过；
  流尾孤立的 `0xAA` 保留；
- 下位机每条上行流独立节拍（频率隔离）：ENC 50Hz、IMU 25Hz、ODOM 10Hz、
  STATUS 1Hz；本程序按自身定时器取最新快照推送（见 protocol-tcp.md）。

## 帧类型

| type | 方向 | 名称 | payload |
|---|---|---|---|
| 0x01 | 板→宿主 | ENC | `int32 count[3]`（累计计数）+ `int16 rpm×10[3]` + `u32 t_ms`（板侧采样时刻，22B；旧固件 18B 无时间戳） |
| 0x02 | 板→宿主 | IMU | `int16 accel_mg[3]` + `int16 gyro_0.01dps[3]` + `int16 rpy_0.01deg[3]` |
| 0x03 | 板→宿主 | STATUS | `u8 uptime, rsv, imu_ok, en, mode, con` |
| 0x04 | 板→宿主 | TELEMETRY | `int16 tgt_rpm×10[3]` + `int16 rpm×10[3]` + `int16 out_pct×10[3]` |
| 0x05 | 板→宿主 | ODOM | `float vx,vy,wz`（车体系）+ `float x,y,yaw`（世界系累计） |
| 0x06 | 双向 | CON | `u8 sender, proto_type, msg_type, proto_ver`（4B） |
| 0x07 | 板→宿主 | SYSID_DATA | `u16 first_idx + u8 n(≤2) + rsv + n×10B 样本`（u32 t_ms + i16 ref/rpm/out ×10；PID 整定数据回传，本程序解析计数不打断链路） |
| 0x08 | 板→宿主 | SYSID_META | `u8 mode, motor, decim, status` + `float kp,ki,kd` + `i16 amp,bias ×100` + `u16 n` + `u32 t_start_ms`（26B，采集起止通告，增益为板上真实值） |
| 0x81 | 宿主→板 | CMD_VEL | `int16 rpm×10[3]`（500ms 超时） |
| 0x82 | 宿主→板 | CMD_STOP | 无（刹车 + 退出闭环 + 中止采集） |
| 0x83 | 宿主→板 | CMD_EN | `u8 en`（仅 1 使能） |
| 0x84 | 宿主→板 | CMD_PID | `float kp, ki, kd` |
| 0x85 | 宿主→板 | CMD_SIGN | `int8 sign[3]` |
| 0x86 | 宿主→板 | CMD_TWIST | `float vx, vy, wz`（500ms 超时） |
| 0x87 | 宿主→板 | CMD_SYSID | `u8 op(0停/1启动/2回传), mode, motor, decim` + `u16 pre_ms, post_ms` + `i16 amp,bias ×100` + `u8 settle_cyc, cycles`（14B；采集测量主通道在 `C30D_Chassis/host_test/tune_session.py`） |
| 0x88 | 宿主→板 | CMD_SYSID_FREQS | `u8 n(≤13) + rsv + u16 centihz[n]`（扫频频率表，整表替换） |
| 0x8F | 宿主→板 | PING | `u32 t_ms`，板原样回显（测 RTT） |

## CON 链路保活（帧 0x06）

上电后串口链路默认未建立，需先建链方可下发运动指令。本程序实现客户端
四态机（DISCONNECTED / CONNECTING / CONNECTED / DISCONNECTING）：

| 参数 | 值 | 说明 |
|---|---|---|
| 建链 | ENABLE_REQUEST → ENABLE_RESPONSE | 应答超时 400ms，重传至多 5 次 |
| 保活 | PING_REQUEST / PING_RESPONSE | 周期 1000ms；未应答 300ms 重发至多 2 次，耗尽断链 |
| 拆链 | DISABLE_REQUEST → DISABLE_RESPONSE | 优雅退出时使用 |
| 断链重连 | 2000ms 节流 | 断链后自动重新建链 |

CON 载荷 4B：`sender`（0x01 板 / 0x02 宿主）、`proto_type`（0x03）、
`msg_type`（0..5 = ENABLE_REQ/RSP、DISABLE_REQ/RSP、PING_REQ/RSP）、
`proto_ver`（0x01）。

## 失效保护层级

| 层级 | 主体 | 行为 |
|---|---|---|
| 1. 链路看门狗（3s） | 下位机 | 3s 收不到宿主 CON 帧 → 判定失联 → 电机停转。本程序以 1s PING 喂狗 |
| 2. 指令超时（500ms） | 下位机 | 速度指令超时目标清零（保持闭环压零）。本程序以 10Hz 重发喂活；控制端断开即停止重发 |
| 3. 控制端断连停车 | 本程序 | 速度指令来源的 TCP 连接断开 → 立即 CMD_STOP |
| 4. 板帧静默告警 | 本程序 | 串口在但 `--rx-timeout-ms`（默认 2s）无任何板帧 → 告警并通知所有 TCP 连接 |
| 5. 设备消失自愈 | 本程序 | 串口设备拔出/消失 → 自动关闭并周期重开，恢复后自动重新建链 |

指令门控：链路非 READY 时拒绝下发 `v/rpm`（登记后待建链生效）；READY 时
自动 CMD_EN；观测到 STATUS `en=0`（下位机重上电）限频 1s 补发；`en 0`
手动失能为粘性状态（重连后保持失能）。
