# TCP 文本协议

> 返回 [README](../README.md)。指令门控、CON 链路保活与失效保护机制见
> [protocol-serial.md](protocol-serial.md)；MATLAB 客户端示例见 `samples/`。

一行一命令（`\n` 结尾，容忍 `\r\n`，`#` 开头为注释行）。应答行以
`ok` / `err` 开头；遥测为 JSON 行。示例：

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

| 命令 | 说明 | 下发帧 |
|---|---|---|
| `v <vx> [vy] [wz]` | 车体速度 m/s、rad/s（登记后由宿主 10Hz 重发） | CMD_TWIST(0x86) |
| `rpm <r1> [r2] [r3]` | 三电机输出轴目标转速 rpm | CMD_VEL(0x81) |
| `stop` | 刹车 + 退出闭环，并停止重发 | CMD_STOP(0x82) |
| `en <0\|1>` | 电机使能。**1**=使能+自动建链；**0**=失能+拆链（粘性，重连后仍失能） | CMD_EN(0x83) |
| `pid <kp> <ki> <kd>` | 三电机共用 PID | CMD_PID(0x84) |
| `sign <s1> <s2> <s3>` | 编码器方向校正 ±1 | CMD_SIGN(0x85) |
| `ping` | 测板端 RTT（下位机原样回显） | PING(0x8F) |
| `watch [on\|off]` | 订阅/退订 JSON 遥测流 | — |
| `stat` `con` `help` `quit` | 统计 / 链路状态 / 帮助 / 断开 | — |

遥测 JSON（`watch` 订阅者）：`odom` 默认 10Hz（`--odom-hz`）；
`enc` / `imu` / `telem` / `status` 默认 1Hz（`--slow-hz`）。
`enc` 行含 `ts` 字段（板侧编码器采样时刻，1ms 时基，新固件 22B 帧）。
`con`（链路状态）与 `serial`（设备 up/down）事件推送给**所有**连接。

多连接：均可 `watch` 观测；速度指令来源（最后一条 `v/rpm` 的连接）断开时
宿主自动 `CMD_STOP` 停车。多控制端并发下发时后到者覆盖，需自行约定。
